#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import os
import rospkg
import airsim
import binvox.binvox as binvox
from scipy.io import savemat
import octomap
from octomap_msgs.msg import Octomap

# 全局变量，但现在将从参数服务器获取实际值
_max_x = None
_max_y = None
_max_z = None
origin_offset = [0.0, 0.0, 0.0]
resolution = 1.0
_local_x = 400
_local_y = 400
_local_z = 400
cruise_height = None  # 新增巡航高度变量

def points_to_gridmap(points, resolution, x_range, y_range):
    """
    将点云数据转换为2D栅格地图
    points: Nx3的点云数据
    resolution: 栅格地图分辨率
    x_range: x轴范围 [min_x, max_x]
    y_range: y轴范围 [min_y, max_y]
    返回：栅格地图（0-100的整数，100表示占据，0表示空闲）
    """
    # 计算地图尺寸
    map_width = int((x_range[1] - x_range[0]) / resolution) + 1
    map_height = int((y_range[1] - y_range[0]) / resolution) + 1
    
    # 创建空地图
    grid_map = np.zeros((map_height, map_width), dtype=np.int8)
    
    # 将点云坐标转换为栅格索引
    x_idx = np.floor((points[:, 0]*resolution - x_range[0]) / resolution).astype(int)
    y_idx = np.floor((points[:, 1]*resolution - y_range[0]) / resolution).astype(int)
    
    # 过滤掉超出范围的点
    valid_idx = (x_idx >= 0) & (x_idx < map_width) & (y_idx >= 0) & (y_idx < map_height)
    x_idx = x_idx[valid_idx]
    y_idx = y_idx[valid_idx]
    
    # 标记占据栅格
    grid_map[y_idx, x_idx] = 100
    
    return grid_map

def get_map_from_airsim(pkg_path):
    global _max_x, _max_y, _max_z
    
    client = airsim.VehicleClient()
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    rospy.logwarn("start the func")
    # using the input center and dim will cause stuck
    # dim = np.array([4000, 4000, 100])

    obstacle_points_whole = np.empty((0, 3))

    for i in range(4):
        center_airsim = airsim.Vector3r(-500.0, -500.0, -50 - 100*i)
        # get voxel grid from simulation
        client.simCreateVoxelGrid(center_airsim, _max_x, _max_y, 100, 5, output_path)
        # read voxel grid data using binvox lib
        convert_data = binvox.Binvox.read(output_path, 'dense', True)
        # convert map to numpy
        map_output = np.zeros_like(convert_data.data, dtype=int)
        map_output[convert_data.data] = 1
        # save the map
        # output_path = os.path.join(pkg_path, 'maps', 'map_output.npy')
        # np.save(output_path, map_output)
        # convet map to point cloud
        obstacle_indices = np.argwhere(map_output == 1)
        obstacle_points = obstacle_indices.astype(float)  # 转换为浮点数坐标
        # delte clout points that below 0 level
        n_x = map_output.shape[0]
        n_y = map_output.shape[1]
        n_z = map_output.shape[2]
        obstacle_points[:, 0] -= n_x/2
        obstacle_points[:, 1] -= n_y/2
        obstacle_points[:, 2] += n_z * i

        # save the point cloud
        # output_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
        # np.save(output_path, filtered_points)
        obstacle_points_whole = np.vstack((obstacle_points_whole, obstacle_points))
    
    min_x = np.min(obstacle_points_whole[:, 0])
    max_x = np.max(obstacle_points_whole[:, 0])

    min_y = np.min(obstacle_points_whole[:, 1])
    max_y = np.max(obstacle_points_whole[:, 1])

    min_z = np.min(obstacle_points_whole[:, 2])
    max_z = np.max(obstacle_points_whole[:, 2])

    rospy.loginfo(f"X range: {min_x} to {max_x}")
    rospy.loginfo(f"Y range: {min_y} to {max_y}")
    rospy.loginfo(f"Z range: {min_z} to {max_z}")
    # obstacle_points_whole = obstacle_points_whole[obstacle_points_whole[:, 2] != 0]
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    os.remove(output_path)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
    np.save(output_path, obstacle_points_whole)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud_mat.mat')
    savemat(output_path, {'data': obstacle_points_whole})
    rospy.logwarn("save map cloud successfully")

def get_map_from_airsim_tiny(pkg_path):
    
    client = airsim.VehicleClient()
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    rospy.logwarn("start the func")


    obstacle_points_whole = np.empty((0, 3))

    for i in range(1):
        time1 = rospy.Time.now()
        center_airsim = airsim.Vector3r(-10, -10, -1)
        # get voxel grid from simulation
        client.simCreateVoxelGrid(center_airsim, 20, 20, 5, 0.05, output_path)
        # read voxel grid data using binvox lib
        convert_data = binvox.Binvox.read(output_path, 'dense', True)
        # convert map to numpy
        map_output = np.zeros_like(convert_data.data, dtype=int)
        map_output[convert_data.data] = 1
        # save the map
        # output_path = os.path.join(pkg_path, 'maps', 'map_output.npy')
        # np.save(output_path, map_output)
        # convet map to point cloud
        obstacle_indices = np.argwhere(map_output == 1)
        obstacle_points = obstacle_indices.astype(float)  # 转换为浮点数坐标
        # delte clout points that below 0 level
        n_x = map_output.shape[0]
        n_y = map_output.shape[1]
        n_z = map_output.shape[2]
        obstacle_points[:, 0] -= n_x/2
        obstacle_points[:, 1] -= n_y/2
        obstacle_points[:, 2] += n_z * i

        # save the point cloud
        # output_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
        # np.save(output_path, filtered_points)
        obstacle_points_whole = np.vstack((obstacle_points_whole, obstacle_points))
        time2 = rospy.Time.now()
        total_time = time2.to_sec() - time1.to_sec()
        total_point = obstacle_points.shape[0]
        rospy.logwarn("loading map uses time: %s s, point number is %s", total_time, total_point )
    

    obstacle_points_whole = obstacle_points_whole[obstacle_points_whole[:, 2] != 0]
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    os.remove(output_path)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
    np.save(output_path, obstacle_points_whole)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud_mat.mat')
    # savemat(output_path, {'data': obstacle_points_whole})
    rospy.logwarn("save map cloud successfully")
    object_list = client.simListSceneObjects()
    print(object_list)
    object_location = client.simGetObjectPose("busingBin2_19")
    print(object_location)
    object_scale = client.simGetObjectScale("busingBin2_19")
    print(object_scale)

def get_local_map_from_airsim(pkg_path):
    client = airsim.VehicleClient()
    output_path = os.path.join(pkg_path, 'maps', 'local_map.binvox')
    rospy.logwarn("get local map")
    # get current local center
    global origin_offset
    state_gt = client.simGetGroundTruthKinematics()
    center_airsim = airsim.Vector3r(state_gt.position.x_val + origin_offset[0],
                                    state_gt.position.y_val + origin_offset[1],
                                    state_gt.position.z_val + origin_offset[2])
    rospy.loginfo("current state: x = %s, y = %s, z = %s", 
                    state_gt.position.x_val,
                    state_gt.position.y_val,
                    state_gt.position.z_val)
    rospy.loginfo("current state in world: x = %s, y = %s, z = %s", 
                    state_gt.position.x_val + origin_offset[0],
                    state_gt.position.y_val + origin_offset[1],
                    state_gt.position.z_val + origin_offset[2])
    # get voxel grid from simulation
    client.simCreateVoxelGrid(center_airsim, _local_x, _local_y, _local_z, resolution, output_path)
    convert_data = binvox.Binvox.read(output_path, 'dense', True)
    map_output = np.zeros_like(convert_data.data, dtype=int)
    map_output[convert_data.data] = 1
    obstacle_indices = np.argwhere(map_output == 1)
    obstacle_points = obstacle_indices.astype(float)  # 转换为浮点数坐标
    n_x = map_output.shape[0]
    n_y = map_output.shape[1]
    n_z = map_output.shape[2]
    obstacle_points[:, 0] -= n_y/2
    obstacle_points[:, 1] -= n_x/2
    obstacle_points[:, 2] -= n_z/2
    obstacle_points[:, 1] += int((state_gt.position.x_val + origin_offset[0])/resolution)
    obstacle_points[:, 0] += int((state_gt.position.y_val + origin_offset[1])/resolution)
    obstacle_points[:, 2] -= int((state_gt.position.z_val + origin_offset[2])/resolution)
    return obstacle_points

def rcv_path_callback(msg: Path):
    pkg_path = rospack.get_path('evtol_airsim_pkg')
    file_name = os.path.join(pkg_path, 'maps', 'path_output.npy')

    # 获取路径中的航路点
    waypoints = msg.poses
    path_len = len(waypoints)
    path_data = np.zeros((path_len, 3))
    

    # 提取航路点坐标并保存为Numpy数组
    for i, pose in enumerate(waypoints):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        path_data[i] = [x, y, z]
    # 保存路径信息为npy文件
    np.save(file_name, path_data)

    rospy.loginfo(f"Saved path to {file_name}")

def read_map_from_local(pkg_path):
    
    client = airsim.VehicleClient()
    output_path = os.path.join(pkg_path, 'maps', 'map_airsim.binvox')
    rospy.logwarn("start the func")
    # using the input center and dim will cause stuck
    # dim = np.array([4000, 4000, 100])

    obstacle_points_whole = np.empty((0, 3))
    convert_data = binvox.Binvox.read(output_path, 'dense', True)
    map_output = np.zeros_like(convert_data.data, dtype=int)
    map_output[convert_data.data] = 1
    obstacle_indices = np.argwhere(map_output == 1)
    obstacle_points = obstacle_indices.astype(float)  # 转换为浮点数坐标
    # delte clout points that below 0 level
    n_x = map_output.shape[0]
    n_y = map_output.shape[1]
    n_z = map_output.shape[2]
    obstacle_points[:, 0] -= n_x/2
    obstacle_points[:, 1] -= n_y/2
    obstacle_points[:, 2] -= n_z/2
    obstacle_points_whole = np.vstack((obstacle_points_whole, obstacle_points))

    min_x = np.min(obstacle_points_whole[:, 0])
    max_x = np.max(obstacle_points_whole[:, 0])

    min_y = np.min(obstacle_points_whole[:, 1])
    max_y = np.max(obstacle_points_whole[:, 1])

    min_z = np.min(obstacle_points_whole[:, 2])
    max_z = np.max(obstacle_points_whole[:, 2])

    rospy.loginfo(f"X range: {min_x} to {max_x}")
    rospy.loginfo(f"Y range: {min_y} to {max_y}")
    rospy.loginfo(f"Z range: {min_z} to {max_z}")
    # obstacle_points_whole = obstacle_points_whole[obstacle_points_whole[:, 2] != 0]
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    # os.remove(output_path)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
    np.save(output_path, obstacle_points_whole)
    output_path = os.path.join(pkg_path, 'maps', 'map_cloud_mat.mat')
    savemat(output_path, {'data': obstacle_points_whole})
    rospy.logwarn("save map cloud successfully")

def points_to_octomap(points, resolution):
    """
    将点云数据转换为Octomap
    points: Nx3的点云数据
    resolution: 八叉树分辨率（米）
    返回：二进制Octomap数据
    """
    # 创建octomap树
    tree = octomap.OcTree(resolution)
    
    # 插入点云数据
    for point in points:
        tree.updateNode(point[0], point[1], point[2], True)
    
    # 更新内部节点
    tree.updateInnerOccupancy()
    
    # 将树转换为二进制数据
    return tree.writeBinary()

if __name__ == '__main__':
    try:
        # 读取地图数据文件的路径
        rospy.init_node('airsim_pub_map_node')
        rospy.loginfo('[airsim_pub_map_node] start')
        
        # 从YAML文件中读取参数
        origin_offset[0] = rospy.get_param("/airsim/offset/x", 0.0)
        origin_offset[1] = rospy.get_param("/airsim/offset/y", 0.0)
        origin_offset[2] = rospy.get_param("/airsim/offset/z", 0.0)
        resolution = rospy.get_param("/airsim/resolution", 5.0)
        
        # 读取地图尺寸参数
        _max_x = rospy.get_param("/airsim/map_size/max_x", 2000)
        _max_y = rospy.get_param("/airsim/map_size/max_y", 2000)
        _max_z = rospy.get_param("/airsim/map_size/max_z", 100)
        
        # 读取巡航高度参数
        cruise_height = rospy.get_param("/flight_param/cruise_height", 100)
        cruise_height_tolerance = rospy.get_param("/flight_param/cruise_height_tolerance", 10.0)  # 高度容差，默认10米
        
        pcl_pub = rospy.Publisher('/airsim/map/point_cloud', PointCloud2, queue_size=10)
        local_map_pub = rospy.Publisher('/airsim/map/local_pcl', PointCloud2, queue_size=10)
        cruise_map_pub = rospy.Publisher('/airsim/map/cruise_pcl', PointCloud2, queue_size=10)  # 新增巡航高度点云发布器
        cruise_grid_pub = rospy.Publisher('/airsim/map/cruise_grid', OccupancyGrid, queue_size=10)  # 新增2D栅格地图发布器
        # octomap_pub = rospy.Publisher('/airsim/map/octomap', Octomap, queue_size=10)  # 新增Octomap发布器
        rospy.Subscriber("/hybrid/path", Path, rcv_path_callback)
        rospy.logwarn("start the node")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('evtol_airsim_pkg')
        get_map_from_airsim(pkg_path)
        
        file_path = os.path.join(pkg_path, 'maps', 'map_cloud.npy')
        cloud_data = np.load(file_path)
        rospy.logwarn("read cloud data")

        # rosinfo cloud_data的范围
        rospy.loginfo(f"cloud_data range: {np.min(cloud_data[:, 0])} to {np.max(cloud_data[:, 0])}, {np.min(cloud_data[:, 1])} to {np.max(cloud_data[:, 1])}, {np.min(cloud_data[:, 2])} to {np.max(cloud_data[:, 2])}")
        
        # 筛选巡航高度附近的点云
        cruise_mask = np.abs(cloud_data[:, 2] - cruise_height/resolution) <= cruise_height_tolerance/resolution
        cruise_cloud_data = cloud_data[cruise_mask]
        rospy.loginfo(f"Total points: {len(cloud_data)}, Cruise height points: {len(cruise_cloud_data)}")
        
        # 创建2D栅格地图
        x_range = [-_max_x/2, _max_x/2]  # 实际世界坐标范围（米）
        y_range = [-_max_y/2, _max_y/2]
        grid_map = points_to_gridmap(cruise_cloud_data, resolution, x_range, y_range)
        
        # 创建OccupancyGrid消息
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = 0.2  # 每个格子的实际大小（米/格）
        grid_msg.info.width = grid_map.shape[1]  # 地图宽度（格子数）
        grid_msg.info.height = grid_map.shape[0]  # 地图高度（格子数）
        
        # 设置地图原点（左下角）在世界坐标系中的位置
        # origin表示栅格地图(0,0)在世界坐标系中的位置（米）
        # 对于地图中任意点(i,j)，其世界坐标为：
        # world_x = origin.position.x + (i * resolution)
        # world_y = origin.position.y + (j * resolution)
        grid_msg.info.origin.position.x = 0.0  # 地图左边界的世界坐标
        grid_msg.info.origin.position.y = 0.0  # 地图下边界的世界坐标
        grid_msg.info.origin.position.z = 0.0  # 巡航高度
        
        # 将2D数组展平为1D列表，按行优先顺序
        # 数据从左下角开始，先行后列
        grid_msg.data = grid_map.flatten().tolist()
        
        head = Header()
        head.stamp = rospy.Time.now()
        head.frame_id = "world"
        cloud_data_whole_msg = create_cloud_xyz32(head, cloud_data)
        cruise_cloud_msg = create_cloud_xyz32(head, cruise_cloud_data)  # 创建巡航高度点云消息


        # # 创建Octomap消息
        # octomap_msg = Octomap()
        # octomap_msg.header.frame_id = "world"
        # octomap_msg.binary = True  # 使用二进制格式
        # octomap_msg.id = "airsim_map"
        # octomap_msg.resolution = resolution
        # octomap_msg.data = points_to_octomap(cloud_data, resolution)
        # it is not efficient in python to generate octomap
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            head.stamp = rospy.Time.now()  # 更新时间戳
            cloud_data_whole_msg.header = head
            cruise_cloud_msg.header = head
            grid_msg.header.stamp = head.stamp
            
            pcl_pub.publish(cloud_data_whole_msg)
            cruise_map_pub.publish(cruise_cloud_msg)  # 发布巡航高度点云
            cruise_grid_pub.publish(grid_msg)  # 发布2D栅格地图
            

            
            # octomap_pub.publish(octomap_msg)  # 发布Octomap
            
            rate.sleep()

    except rospy.ROSInterruptException:
        pass