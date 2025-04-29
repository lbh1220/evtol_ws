#!/usr/bin/env python

import rospy
import airsim
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import nav_msgs.msg
from nav_msgs.msg import Path
import os
import rospkg
import geometry_msgs.msg
import json
import TrajectoryPlanner as TP
client = airsim.MultirotorClient()
origin_offset = [0.0, 0.0, 0.0]
resolution = 5.0
has_offset = False
def execute_profile(waypoints_data):
    time.sleep(5)
    # get a list for path
    global has_offset
    global origin_offset
    
    # origin_offset = [0, 0, -50]
    origin_offset_msg = geometry_msgs.msg.Point()
    origin_offset_msg.x = origin_offset[0]
    origin_offset_msg.y = origin_offset[1]
    origin_offset_msg.z = origin_offset[2]
    len_path = waypoints_data.shape[0]
    path_wp = [np.zeros([1, 3])] * len_path
    for i in range(len_path):
        path_wp[i] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                            waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                            -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]])
        
    # connect to airsim
    
    client.confirmConnection()
    client.simFlushPersistentMarkers()
    client.enableApiControl(True)
    client.armDisarm(True)

    # to finish the profile
    client.takeoffAsync().join()
    start_state = client.simGetGroundTruthKinematics()


    state_ref = np.array([start_state.position.x_val, start_state.position.y_val, start_state.position.z_val])
    start_point = state_ref - np.array([0, 0, 0.5]) # 把起点向上稍微提升一点点

    path = []
    path_inWord = []
    for i in range(len_path):
        path.append(airsim.Vector3r(path_wp[i][0], path_wp[i][1], path_wp[i][2]))
        path_inWord.append(path[i] + airsim.Vector3r(origin_offset[0], origin_offset[1], origin_offset[2]))

    client.simPlotLineStrip(path_inWord, [0.0, 1.0, 0.0, 1.0], is_persistent=True)

    drivetrain1 = airsim.DrivetrainType.ForwardOnly
    yaw_mode1 = airsim.YawMode(False, 0)
    rospy.logwarn("execute mission profile")
    pub_start_flag.publish(origin_offset_msg)
    
    client.moveOnPathAsync(path, 15, drivetrain=drivetrain1, yaw_mode=yaw_mode1).join()
    # client.moveOnPathAsync(path, 10).join()
    pub_stop_flag.publish(origin_offset_msg)
    client.hoverAsync().join()
    time.sleep(5)
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)



def execute_profile_from_cruise(waypoints_data):
    # time.sleep(5)
    # get a list for path
    global has_offset
    global origin_offset
    
    # origin_offset = [0, 0, -50]
    origin_offset_msg = geometry_msgs.msg.Point()
    origin_offset_msg.x = origin_offset[0]
    origin_offset_msg.y = origin_offset[1]
    origin_offset_msg.z = origin_offset[2]
    len_path = waypoints_data.shape[0]
    path_wp = [np.zeros([1, 3])] * len_path
    count = 0
    for i in range(len_path):
        h_cruise = -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]
        if(h_cruise <= -20.00):
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -20.00])
        else:
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]])    
        count = count + 1        

    
    client.confirmConnection()
    client.simFlushPersistentMarkers()
    client.enableApiControl(True)
    client.armDisarm(True)

    # to finish the profile
    client.takeoffAsync().join()
    start_state = client.simGetGroundTruthKinematics()
    

    state_ref = np.array([start_state.position.x_val, start_state.position.y_val, start_state.position.z_val])
    start_point = state_ref - np.array([0, 0, 0.5]) # 把起点向上稍微提升一点点

    path = []
    path_inWord = []
    for i in range(count):
        path.append(airsim.Vector3r(path_wp[i][0], path_wp[i][1], path_wp[i][2]))
        path_inWord.append(path[i] + airsim.Vector3r(origin_offset[0], origin_offset[1], origin_offset[2]))

    client.simPlotLineStrip(path_inWord, [0.0, 1.0, 0.0, 1.0], is_persistent=True)

    drivetrain1 = airsim.DrivetrainType.ForwardOnly
    yaw_mode1 = airsim.YawMode(False, 0)
    rospy.logwarn("execute mission profile")
    pub_start_flag.publish(origin_offset_msg)
    # client.moveToZAsync(-149.99999999, 10).join()
    # client.hoverAsync().join()
    # time.sleep(10)
    
    # client.moveOnPathAsync(path, 10, drivetrain=drivetrain1, yaw_mode=yaw_mode1).join()
    client.moveOnPathAsync(path, 2).join()


    pub_stop_flag.publish(origin_offset_msg)
    client.hoverAsync().join()
    time.sleep(5)
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
def execute_profile_square():
    time.sleep(5)
    # get a list for path

    waypoints_data = np.array([[0.0, 0.0, 1.0],
                               [5.0, 0.0, 1.5],
                               [5.0, 5.0, 2.0],
                               [0.0, 5.0, 1.5],
                               [0.0, 0.0, 1.0]])
    global has_offset
    global origin_offset
    
    # origin_offset = [0, 0, -50]
    origin_offset_msg = geometry_msgs.msg.Point()
    origin_offset_msg.x = origin_offset[0]
    origin_offset_msg.y = origin_offset[1]
    origin_offset_msg.z = origin_offset[2]
    len_path = waypoints_data.shape[0]
    path_wp = [np.zeros([1, 3])] * len_path
    count = 0
    for i in range(len_path):
        h_cruise = -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]
        if(h_cruise <= -10000.00):
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -60.00])
        else:
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]])    
        count = count + 1        
            
    
    client.confirmConnection()
    client.simFlushPersistentMarkers()
    client.enableApiControl(True)
    client.armDisarm(True)

    # to finish the profile
    client.takeoffAsync().join()
    start_state = client.simGetGroundTruthKinematics()
    

    state_ref = np.array([start_state.position.x_val, start_state.position.y_val, start_state.position.z_val])
    start_point = state_ref - np.array([0, 0, 0.5]) # 把起点向上稍微提升一点点

    path = []
    path_inWord = []
    for i in range(count):
        path.append(airsim.Vector3r(path_wp[i][0], path_wp[i][1], path_wp[i][2]))
        path_inWord.append(path[i] + airsim.Vector3r(origin_offset[0], origin_offset[1], origin_offset[2]))

    client.simPlotLineStrip(path_inWord, [0.0, 1.0, 0.0, 1.0], is_persistent=True)

    drivetrain1 = airsim.DrivetrainType.ForwardOnly
    yaw_mode1 = airsim.YawMode(False, 0)
    rospy.logwarn("execute mission profile")
    pub_start_flag.publish(origin_offset_msg)
    # client.moveToZAsync(-149.99999999, 10).join()
    # client.hoverAsync().join()
    # time.sleep(10)
    
    client.moveOnPathAsync(path, 1).join()
    # client.moveOnPathAsync(path, 10).join()


    pub_stop_flag.publish(origin_offset_msg)
    client.hoverAsync().join()
    time.sleep(5)
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
def execute_profile_circle():
    time.sleep(5)
    # get a list for path

    waypoints_data = np.array([[0.0, 0.0, 1.0],
                               [5.0, 0.0, 1.5],
                               [5.0, 5.0, 2.0],
                               [0.0, 5.0, 1.5],
                               [0.0, 0.0, 1.0]])
    global has_offset
    global origin_offset
    
    client.confirmConnection()
    client.simFlushPersistentMarkers()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-3, 1).join()
    center = np.array([[0], [0]])
    speed = 2  # 速度设置
    radius = 5  # 半径设置
    clock_wise = True 

    pos_reserve = np.array([[0.], [0.], [-3.]])
    rate = rospy.Rate(50)
    # 速度控制
    while not rospy.is_shutdown():
        # 获取无人机当前位置
        state = client.simGetGroundTruthKinematics()
        pos = np.array([[state.position.x_val], [state.position.y_val], [state.position.z_val]])
        # 计算径向速度的方向向量
        dp = pos[0:2] - center
        if np.linalg.norm(dp) - radius > 0.1:
            vel_dir_1 = -dp
        elif np.linalg.norm(dp) - radius < 0.1:
            vel_dir_1 = dp
        # 计算切向速度的方向向量i
        theta = math.atan2(dp[1, 0], dp[0, 0])
        if clock_wise:
            theta += math.pi / 2
        else:
            theta -= math.pi / 2
        v_dir_2 = np.array([[math.cos(theta)], [math.sin(theta)]])
        # 计算最终速度的方向向量
        v_dir = 0.08 * vel_dir_1 + v_dir_2
        # 计算最终速度指令
        v_cmd = speed * v_dir / np.linalg.norm(v_dir)
        # 速度控制
        client.moveByVelocityZAsync(v_cmd[0, 0], v_cmd[1, 0], -3, 1)
        # 画图
        point_reserve = [airsim.Vector3r(pos_reserve[0, 0], pos_reserve[1, 0], pos_reserve[2, 0])]
        point = [airsim.Vector3r(pos[0, 0], pos[1, 0], pos[2, 0])]
        point_end = pos + np.vstack((v_cmd, np.array([[0]])))
        point_end = [airsim.Vector3r(point_end[0, 0], point_end[1, 0], point_end[2, 0])]
        client.simPlotArrows(point, point_end, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
        client.simPlotLineList(point_reserve + point, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
        # 循环
        pos_reserve = pos
        rate.sleep()

    client.hoverAsync().join()
    time.sleep(5)
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)

def execute_profile_square_head():
    time.sleep(5)
    # get a list for path

    waypoints_data = np.array([[0.0, 0.0, 11.0],
                               [0.0, -400.0, 11.0],
                               [-400.0, -400.0, 11.0],
                               [-400.0, 0.0, 11.0],
                               [0.0, 0.0, 11.0]])
    global has_offset
    global origin_offset
    
    start_point = waypoints_data[1, :]
    len_path = waypoints_data.shape[0]
    goal_point = waypoints_data[len_path - 1, :]

    Planner = TP.TrajectoryPlanner(0, start_point, goal_point, -2.0)
    Planner.update_waypoints(waypoints_data)
    Planner.smooth_path(2.0)
    Traj = Planner.spath
    


    # origin_offset = [0, 0, -50]
    origin_offset_msg = geometry_msgs.msg.Point()
    origin_offset_msg.x = origin_offset[0]
    origin_offset_msg.y = origin_offset[1]
    origin_offset_msg.z = origin_offset[2]
    
    path_wp = [np.zeros([1, 3])] * len_path
    count = 0
    for i in range(len_path):
        h_cruise = -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]
        if(h_cruise <= -10000.00):
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -60.00])
        else:
            path_wp[count] = np.array([waypoints_data[len_path - i - 1, 1] * resolution - origin_offset[1],
                                waypoints_data[len_path - i - 1, 0] * resolution - origin_offset[0],
                                -waypoints_data[len_path - i - 1, 2] * resolution - origin_offset[2]])    
        count = count + 1        
            
    
    client.confirmConnection()
    client.simFlushPersistentMarkers()
    client.enableApiControl(True)
    client.armDisarm(True)

    # to finish the profile
    client.takeoffAsync().join()
    start_state = client.simGetGroundTruthKinematics()
    

    state_ref = np.array([start_state.position.x_val, start_state.position.y_val, start_state.position.z_val])
    start_point = state_ref - np.array([0, 0, 0.5]) # 把起点向上稍微提升一点点

    path = []
    path_inWord = []
    for i in range(count):
        path.append(airsim.Vector3r(path_wp[i][0], path_wp[i][1], path_wp[i][2]))
        path_inWord.append(path[i] + airsim.Vector3r(origin_offset[0], origin_offset[1], origin_offset[2]))
    
    client.simPlotLineStrip(path_inWord, [0.0, 1.0, 0.0, 1.0], is_persistent=True)

    drivetrain1 = airsim.DrivetrainType.ForwardOnly
    yaw_mode1 = airsim.YawMode(False, 0)
    rospy.logwarn("execute mission profile")
    pub_start_flag.publish(origin_offset_msg)
    # client.moveToZAsync(-149.99999999, 10).join()
    # client.hoverAsync().join()
    # time.sleep(10)
    
    client.moveOnPathAsync(path, 10).join()
    # client.moveOnPathAsync(path, 10).join()


    pub_stop_flag.publish(origin_offset_msg)
    client.hoverAsync().join()
    time.sleep(5)
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)



def rcv_mission_callback(msg: Path):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('evtol_airsim_pkg')
    file_name = os.path.join(pkg_path, 'maps', 'path_output.npy')

    # 获取路径中的航路点
    waypoints = msg.poses
    path_len = len(waypoints)
    waypoints_data = np.zeros((path_len, 3))
    # 提取航路点坐标并保存为Numpy数组
    for i, pose in enumerate(waypoints):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        waypoints_data[i] = [x, y, z]
    # execute_profile(waypoints_data)
    


def get_profile_from_file():

    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('evtol_airsim_pkg')
    file_name = os.path.join(pkg_path, 'maps', 'path_output.npy')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if os.path.exists(file_name):
            waypoints_data = np.load(file_name)
            # time.sleep(50)
            # rospy.signal_shutdown("finish mission")
            execute_profile(waypoints_data)
            # execute_profile_square_head()

            break

        else:
            rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('airsim_mission_node')
        rospy.loginfo('[airsim_mission_node] start')
        pub_start_flag = rospy.Publisher('/airsim/mission/start_flag', geometry_msgs.msg.Point, queue_size=10)
        pub_stop_flag = rospy.Publisher('/airsim/mission/stop_flag', geometry_msgs.msg.Point, queue_size=10)
        origin_offset[0] = rospy.get_param("airsim/offset_x")
        origin_offset[1] = rospy.get_param("airsim/offset_y")
        origin_offset[2] = rospy.get_param("airsim/offset_z")
        resolution = rospy.get_param("airsim/resolution")


        
        get_profile_from_file()
        rospy.Subscriber("/hybrid/path", Path, rcv_mission_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass       