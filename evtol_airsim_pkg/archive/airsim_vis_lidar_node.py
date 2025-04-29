#!/usr/bin/env python

import rospy
import airsim
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2

def publish_lidar_data(client,my_lidar_name):
    topic_name = '/airsim/lidar/point_cloud/' + my_lidar_name
    pub_lidar = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    Lidar_data_raw = client.getLidarData(lidar_name = my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))
        # points_position[:, 2] = -points_position[:, 2]

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        lidar_msg = pc2.create_cloud_xyz32(header, points_position)

        pub_lidar.publish(lidar_msg)


if __name__ == '__main__':
    try:

        rospy.init_node('airsim_vis_lidar_node', anonymous=True)
        rospy.loginfo('[airsim_vis_lidar_node] start')
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