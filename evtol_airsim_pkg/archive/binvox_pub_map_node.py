#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from nav_msgs.msg import Path
import os
import rospkg
import airsim
import binvox.binvox as binvox
from scipy.io import savemat
_max_x = 1000
_max_y = 1000
_max_z = 100
origin_offset = [0.0, 0.0, 0.0]
resolution = 1.0
_local_x = 400
_local_y = 400
_local_z = 400
def get_map_from_airsim(pkg_path):
    
    client = airsim.VehicleClient()
    output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    rospy.logwarn("start the func")
    # using the input center and dim will cause stuck
    # dim = np.array([4000, 4000, 100])

    obstacle_points_whole = np.empty((0, 3))

    for i in range(4):
        center_airsim = airsim.Vector3r(0.0, 0.0, -50 - 100*i)
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

def read_map_from_local(pkg_path, file_name):
    
    client = airsim.VehicleClient()
    # output_path = os.path.join(pkg_path, 'maps', 'map_airsim.binvox')
    output_path = os.path.join(pkg_path, 'maps', file_name+'.binvox')
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

    min_z_threshold = -2
    obstacle_points_whole = obstacle_points_whole[obstacle_points_whole[:, 2] >= min_z_threshold]

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
    # output_path = os.path.join(pkg_path, 'maps', 'map.binvox')
    # os.remove(output_path)
    output_path = os.path.join(pkg_path, 'maps', file_name+'.npy')
    np.save(output_path, obstacle_points_whole)
    # output_path = os.path.join(pkg_path, 'maps', file_name+'.mat')
    # savemat(output_path, {'data': obstacle_points_whole})
    rospy.logwarn("save map cloud successfully")


if __name__ == '__main__':
    try:
        # 读取地图数据文件的路径
        rospy.init_node('binvox_pub_map_node')
        rospy.loginfo('[binvox_pub_map_node] start')
        origin_offset[0] = rospy.get_param("airsim/offset_x")
        origin_offset[1] = rospy.get_param("airsim/offset_y")
        origin_offset[2] = rospy.get_param("airsim/offset_z")
        resolution = rospy.get_param("airsim/resolution")
        pcl_pub = rospy.Publisher('/airsim/map/point_cloud', PointCloud2, queue_size=10)
        local_map_pub = rospy.Publisher('/airsim/map/local_pcl', PointCloud2, queue_size=10)
        rospy.Subscriber("/hybrid/path", Path, rcv_path_callback)
        rospy.logwarn("start the node")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('evtol_airsim_pkg')
        # get_map_from_airsim(pkg_path)
        file_name = 'hdb_d512_e_cb'
        read_map_from_local(pkg_path, file_name)
        
        # save_map(pkg_path)
        file_path = os.path.join(pkg_path, 'maps', file_name+'.npy')
        cloud_data = np.load(file_path)
        rospy.logwarn("read cloud data")
        head = Header()
        head.stamp = rospy.Time.now()
        head.frame_id = "world"
        cloud_data_whole_msg = create_cloud_xyz32(head, cloud_data)

        target_z = 1  # 特定高度 z，默认为 0.0
        delta_z = 0.5  # z 范围容差，默认为 0.5

        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pcl_pub.publish(cloud_data_whole_msg)
            # 筛选特定高度的点云
            mask = (cloud_data[:, 2] >= target_z - delta_z) & (cloud_data[:, 2] <= target_z + delta_z)
            local_cloud_data = cloud_data[mask]

            # 创建局部点云消息
            cloud_data_local_msg = create_cloud_xyz32(head, local_cloud_data)
            cloud_data_local_msg.header.stamp = rospy.Time.now()
            cloud_data_local_msg.header.frame_id = "world"

            # 发布局部点云
            local_map_pub.publish(cloud_data_local_msg)

            rate.sleep()
            

        

    except rospy.ROSInterruptException:
        pass