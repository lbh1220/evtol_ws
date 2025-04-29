"""
轨迹规划的类
"""

import numpy as np
import math


class TrajectoryPlanner:
    def __init__(self, map_data, start, goal, cruise_height):
        self.map_data = map_data  # 传入地图数据
        self.start = start
        self.goal = goal
        self.waypoint_list = []
        self.path = []
        self.spath = []
        self.g_acc = 9.78030
        self.cruise_height = cruise_height
        self.climb_angle = np.deg2rad(60.0)
        self.descend_angle = np.deg2rad(60.0)

    # update waypoints
    def update_waypoints(self, waypoints):
        self.waypoint_list = []
        self.waypoint_list = waypoints
        self.path = []
        # **********最好在这里检查一下是不是有waypoint离起点终点太近了
        self.path = [self.start] + self.waypoint_list + [self.goal]
        return self.waypoint_list

    # python似乎不需要这种操作，可以直接在外面访问成员变量
    def get_path(self):
        return self.path

    def smooth_path(self, speed):
        turn_radius = self.get_radius(speed, 0.0, 0.0)
        self.spath = [self.path[0]]
        curr_point = self.path[0]
        for i in range(len(self.path) - 2):# 如果有N个点，一共构成N-1段，他们有N-2个过渡段
            arc_waypoints = self.get_arc_waypoints(curr_point, self.path[i+1], self.path[i+2], turn_radius)
            self.spath = self.spath + arc_waypoints
            curr_point = arc_waypoints[-1]

        self.spath = self.spath + [self.path[-1]] +[self.path[-1]]

    def generate_trajcetory_3D(self):
        # *************暂定*************
        # 沿着path（2维）构造一个3D的路径出来
        for i in range(len(self.path)):
            if 0 < i < (len(self.path) - 1):
                self.path[i][2] = self.cruise_height
        self.path[0][2] = self.start[2]
        self.path[-1][2] = self.goal[2]
        dist_temp = 0.0
        # z轴的正方向是向下的
        dist_climb = np.abs(self.cruise_height - self.start[2]) / np.tan(self.climb_angle)
        dist_descent = np.abs(self.cruise_height - self.goal[2]) / np.tan(self.descend_angle)
        dist_temp = 0.0

        for i in range(len(self.path) - 1):
            dist_temp = dist_temp + self.get_dist_2d(self.path[i], self.path[i + 1])
        if dist_temp > (dist_climb + dist_descent):
            dist_temp = 0.0
            for i in range(len(self.path) - 1):
                dist_temp = dist_temp + self.get_dist_2d(self.path[i], self.path[i + 1])
                if dist_temp > dist_climb:
                    coorS = np.array([self.path[i + 1][0], self.path[i + 1][1], self.path[i + 1][2]])
                    coorS[2] = self.cruise_height
                    coorT = np.array([self.path[i][0], self.path[i][1], self.path[i][2]])
                    coorT[2] = self.cruise_height
                    coor_TOC = self.get_line_coor_dist(coorS, coorT, (dist_temp - dist_climb))
                    self.path.insert(i + 1, coor_TOC)
                    break
            dist_temp = 0.0
            len_curr = len(self.path)
            for i in range(len_curr - 1):
                dist_temp = dist_temp + self.get_dist_2d(self.path[len_curr - 1 - i], self.path[len_curr - 2 - i])
                if dist_temp > dist_descent:
                    coorS = np.array([self.path[len_curr - 2 - i][0], self.path[len_curr - 2 - i][1], self.path[len_curr - 2 - i][2]])
                    coorS[2] = self.cruise_height
                    coorT = np.array([self.path[len_curr - 1 - i][0], self.path[len_curr - 1 - i][1], self.path[len_curr - 1 - i][2]])
                    coorT[2] = self.cruise_height
                    coor_TOD = self.get_line_coor_dist(coorS, coorT, (dist_temp - dist_descent))
                    self.path.insert(len_curr - 1 - i, coor_TOD)
                    break
                    # ********本段需要考虑可能出现的TOD或TOC和某一个现有waypoint重合的问题
            return self.path
        else:
            # 如果航程不够，按理说应该以更陡的角度来爬升，但是先算了
            pass

        return  self.path




    # 根据速度speed，航向trk和转弯方向来计算转弯半径
    def get_radius(self, speed, trk, dir):
        r = speed * speed / self.g_acc / np.tan(np.deg2rad(10))
        return r

    def get_arc_waypoints(self, coorS, coorM, coorD, R):
        SM_direction = (coorM - coorS) / np.linalg.norm(coorM - coorS)
        MD_direction = (coorD - coorM) / np.linalg.norm(coorD - coorM)

        SM_length = np.linalg.norm(coorM - coorS)
        MD_length = np.linalg.norm(coorD - coorM)
        turn_dir = 1  # 1表示顺时针，-1表示逆时针
        if np.cross(SM_direction, MD_direction)[2] > 0:
            turn_dir = 1
        else:
            turn_dir = -1

        # 转弯角度应该在0到180之间的
        # turn_angle = np.arccos(np.dot(SM_direction, MD_direction))
        # 上面那样写数值稳定性不好
        cos_value = np.clip(np.dot(SM_direction, MD_direction), -1, 1)
        turn_angle = np.arccos(cos_value)
        # 转弯的提前量是多少
        len_lookhead = R * np.tan(turn_angle / 2.0)
        len_lookhead = np.abs(len_lookhead)
        
        
        arc_waypoints = []
        if SM_length >= len_lookhead and MD_length >= len_lookhead:
            coor_turn_start = self.get_line_coor_dist(coorM, coorS, len_lookhead)
            coor_turn_end = self.get_line_coor_dist(coorM, coorD, len_lookhead)
            SO_dir = self.rotate_vector_around_plane(SM_direction, MD_direction, SM_direction, math.pi / 2.0)
            coor_turn_center = self.get_line_coor_dir(coor_turn_start, SO_dir, R)
            # arc_waypoints = [coor_turn_start]
            dir_ref = - SO_dir
            delta_angle = turn_angle / 10.0
            for i in range(10):
                dir_ref = self.rotate_vector_around_plane(SM_direction, MD_direction, - SO_dir, i * delta_angle)
                coor_ref = self.get_line_coor_dir(coor_turn_center, dir_ref, R)
                arc_waypoints.append(coor_ref)
            arc_waypoints.append(coor_turn_end)

        else:
            arc_waypoints = [coorM]

        return  arc_waypoints
            
            

    def plan_path(self, start, goal):
        # 实现路径规划算法，返回规划路径
        # 可以使用A*、Dijkstra、RRT等算法
        # 使用self.map_data来查询地图信息
        pass

    def visualize_path(self, path):
        # 将规划的路径可视化，用于调试和展示
        pass

    def execute_path(self, path):
        # 执行路径，控制机器人按照路径移动
        pass

    def update_map(self, new_map_data):
        # 更新地图数据，例如遇到动态障碍物
        self.map_data = new_map_data

    def rotate_vector_around_plane(self, S, V, dir_ref, theta):
        # 在S和V组成的平面中，按照右手定则旋转theta（弧度制）
        # 计算垂直于S和V所确定平面的法线向量N
        N = np.cross(S, V)

        # 计算旋转矩阵
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotation_matrix = np.array([[cos_theta + N[0] ** 2 * (1 - cos_theta),
                                     N[0] * N[1] * (1 - cos_theta) - N[2] * sin_theta,
                                     N[0] * N[2] * (1 - cos_theta) + N[1] * sin_theta],
                                    [N[1] * N[0] * (1 - cos_theta) + N[2] * sin_theta,
                                     cos_theta + N[1] ** 2 * (1 - cos_theta),
                                     N[1] * N[2] * (1 - cos_theta) - N[0] * sin_theta],
                                    [N[2] * N[0] * (1 - cos_theta) - N[1] * sin_theta,
                                     N[2] * N[1] * (1 - cos_theta) + N[0] * sin_theta,
                                     cos_theta + N[2] ** 2 * (1 - cos_theta)]])

        # 使用旋转矩阵将向量S绕法线N旋转theta度
        rotated_S = np.dot(rotation_matrix, dir_ref)

        return rotated_S

    def get_line_coor_dist(self, coorS, coorT, dist):
        # 计算从S指向T的直线上，距离S为dist的点的坐标
        direction = coorT - coorS

        # 计算目标点的坐标
        target_point = coorS + (direction / np.linalg.norm(direction)) * dist

        # 检查目标点是否在线段 AB 之间
        # if 0 <= np.dot(target_point - coorT, direction) <= np.dot(coorS - coorT, direction):
        #     in_segment = True
        # else:
        #     in_segment = False

        return target_point

    def get_line_coor_dir(self, coorS, dir, dist):
        # 计算从S指向dir的直线上，距离S为dist的点的坐标
        target_point = coorS + (dir / np.linalg.norm(dir)) * dist

        return target_point
    def get_dist_2d(self, coorS, coorT):
        # 计算从S到T之间的二维路径的长度
        dist = np.linalg.norm(np.array([coorS[0] - coorT[0], coorS[1] - coorT[1]]))
        return dist