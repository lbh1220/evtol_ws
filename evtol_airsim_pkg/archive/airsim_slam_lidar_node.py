#!/usr/bin/env python

import rospy
import airsim
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import nav_msgs.msg
import sensor_msgs.point_cloud2 as pc2

def publish_lidar_data(client,my_lidar_name):
    topic_name = '/airsim/lidar/point_cloud/' + my_lidar_name
    pub_lidar = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    Lidar_data_raw = client.getLidarData(lidar_name = my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))
        pcl_body = np.zeros(shape=points_position.shape)
        # # pcl_body[:, 0] = -points_position[:, 1]  # x坐标
        # # pcl_body[:, 1] = points_position[:, 0]  # y坐标
        # # pcl_body[:, 2] = points_position[:, 2] # z坐标
        # pcl_body[:, 0] = points_position[:, 0]  # x坐标
        # pcl_body[:, 1] = points_position[:, 1]  # y坐标
        # pcl_body[:, 2] = points_position[:, 2] # z坐标
        # state = client.simGetGroundTruthKinematics()
        # w = state.orientation.w_val
        # x = state.orientation.x_val
        # y = state.orientation.y_val
        # z = state.orientation.z_val
        # rotation_matrix = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        #                     [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        #                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        # pcl_body = pcl_body.T
        # pcl_world = np.dot(rotation_matrix, pcl_body)
        # pcl_world = pcl_world.T
        # pcl_world[:, 0] = pcl_world[:, 0] + state.position.x_val
        # pcl_world[:, 1] = pcl_world[:, 1] + state.position.y_val
        # pcl_world[:, 2] = pcl_world[:, 2] + state.position.z_val
        #################################
        # there must be something going wrong, getLidarData() output the cloud on Vehicle frame
        # But the vehicle is moving, how could it be right without add vehicle's position
        ################################
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        lidar_msg = pc2.create_cloud_xyz32(header, points_position)

        pub_lidar.publish(lidar_msg)

    

if __name__ == '__main__':
    try:

        rospy.init_node('airsim_slam_lidar_node', anonymous=True)
        rospy.loginfo('[airsim_slam_lidar_node] start')
        rate = rospy.Rate(10)

        client = airsim.MultirotorClient()
        client.confirmConnection()

        while not rospy.is_shutdown():
            publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_left')
            publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_front')
            publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_right')
            publish_lidar_data(client=client, my_lidar_name='Lidar_mid360_bottom')

            rate.sleep()

    except rospy.ROSInterruptException:
        pass