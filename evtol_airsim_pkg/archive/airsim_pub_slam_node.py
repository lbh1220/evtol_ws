#!/usr/bin/env python

import rospy
import airsim
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import nav_msgs.msg
import geometry_msgs.msg

def get_lidar_data(client,my_lidar_name):

    Lidar_data_raw = client.getLidarData(lidar_name = my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))
        # points_position[:, 2] = -points_position[:, 2]
        return points_position

if __name__ == '__main__':
    try:

        rospy.init_node('airsim_pub_slam_node', anonymous=True)
        rospy.loginfo('[airsim_pub_slam_node] start')

        pub_lidar = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        pub_imu = rospy.Publisher('/imu/data', nav_msgs.msg.Odometry, queue_size=10)
        rate = rospy.Rate(10)

        client = airsim.MultirotorClient()
        client.confirmConnection()

        while not rospy.is_shutdown():
            point1 = get_lidar_data(client=client, my_lidar_name='Lidar_Avia_left')
            point2 = get_lidar_data(client=client, my_lidar_name='Lidar_Avia_front')
            point3 = get_lidar_data(client=client, my_lidar_name='Lidar_Avia_right')
            point4 = get_lidar_data(client=client, my_lidar_name='Lidar_mid360_bottom')
            pcl_lidar = np.concatenate((point1, point2, point3, point4), axis=0)

            state = client.simGetGroundTruthKinematics()

            
            header_common = std_msgs.msg.Header()
            header_common.stamp = rospy.Time.now()
            header_common.frame_id = "world"
            lidar_msg = pc2.create_cloud_xyz32(header_common, pcl_lidar)

            state_msg = nav_msgs.msg.Odometry()
            state_msg.header = header_common
            state_msg.pose.pose.position.x = state.position.x_val
            state_msg.pose.pose.position.y = state.position.y_val
            state_msg.pose.pose.position.z = state.position.z_val
            state_msg.pose.pose.orientation.x = state.orientation.x_val
            state_msg.pose.pose.orientation.y = state.orientation.y_val
            state_msg.pose.pose.orientation.z = state.orientation.z_val
            state_msg.pose.pose.orientation.w = state.orientation.w_val

            pub_imu.publish(state_msg)
            pub_lidar.publish(lidar_msg)





            rate.sleep()

    except rospy.ROSInterruptException:
        pass