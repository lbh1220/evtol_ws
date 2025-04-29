#!/usr/bin/env python

import rospy
import airsim
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from mavros_msgs.srv import MessageInterval

def publish_lidar_data(client, my_lidar_name, transformation_matrix):
    topic_name = '/airsim/lidar/point_cloud/' + my_lidar_name
    pub_lidar = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    Lidar_data_raw = client.getLidarData(lidar_name=my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))

        # Add a column of ones to the points_position array to make it homogeneous coordinates
        ones = np.ones((points_position.shape[0], 1))
        homogeneous_points = np.hstack((points_position, ones))

        # gazebo和airsim的机体坐标系差180度
        transformed_points = np.dot(homogeneous_points, transformation_matrix.T)
        transformation_gazebo_body = np.array([
            [1, 0, 0, 0],  
            [0, -1, 0, 0],   
            [0, 0, -1, 0],  
            [0, 0, 0, 1] 
        ])
        transformed_points = np.dot(transformed_points, transformation_gazebo_body.T)
        # transformed_points = np.dot(transformation_matrix, homogeneous_points.T)
        # transformed_points = transformed_points.T
        # Remove the homogeneous coordinate before creating the point cloud
        points_position_transformed = transformed_points[:, :3]

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "body"

        lidar_msg = pc2.create_cloud_xyz32(header, points_position_transformed)
        pub_lidar.publish(lidar_msg)


def set_message_interval():

    rospy.wait_for_service('/mavros/set_message_interval')
    try:
        set_interval = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)
        resp = set_interval(105, 1000)
        print("Set /mavros/imu/data_raw message interval: %s" % resp.success)
        resp = set_interval(31, 1000)
        print("Set /mavros/imu/data message interval: %s" % resp.success)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        rospy.init_node('airsim_lidar_body_node', anonymous=True)
        rospy.loginfo('[airsim_lidar_body_node] start')
        # set_message_interval()
        rate = rospy.Rate(10)

        client = airsim.MultirotorClient()
        client.confirmConnection()
        sinA = np.sin(70*np.pi / 180.0)
        cosA = np.cos(70*np.pi / 180.0)
        # Define the transformation matrix
        transformation_front = np.array([
            [1, 0, 0, 3],  # x方向前进
            [0, 1, 0, 0],   
            [0, 0, 1, 0],  
            [0, 0, 0, 1] 
        ])
        transformation_left = np.array([
            [cosA, sinA, 0, 0],   
            [-sinA, cosA, 0, -1],   
            [0, 0, 1, 0],  
            [0, 0, 0, 1] 
        ])
        transformation_right = np.array([
            [cosA, -sinA, 0, 0],   
            [sinA, cosA, 0, 1],   
            [0, 0, 1, 0],  
            [0, 0, 0, 1] 
        ])
        transformation_bottom = np.array([
            [1, 0, 0, 0],   
            [0, 1, 0, 0],   
            [0, 0, 1, 0.5],  
            [0, 0, 0, 1] 
        ])
        while not rospy.is_shutdown():
            publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_front', transformation_matrix=transformation_front)
            publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_front_2', transformation_matrix=transformation_front)
            # publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_front_360', transformation_matrix=transformation_front)
            # publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_front_180', transformation_matrix=transformation_front)

            # publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_left', transformation_matrix=transformation_left)    
            # publish_lidar_data(client=client, my_lidar_name='Lidar_Avia_right', transformation_matrix=transformation_right)

            publish_lidar_data(client=client, my_lidar_name='Lidar_mid360_bottom', transformation_matrix=transformation_bottom)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
