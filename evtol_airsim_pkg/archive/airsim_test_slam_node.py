#!/usr/bin/env python

import rospy
import airsim
import numpy as np
import math
import time
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

import sensor_msgs.point_cloud2 as pc2

import nav_msgs.msg
import geometry_msgs.msg

PointsPerSecond = 600000
NumberOfChannels = 64
VerticalFOVUpper = 40.0
VerticalFOVLower = -40.0
# PointsPerSecond = 288000
# NumberOfChannels = 16
# VerticalFOVUpper = 15
# VerticalFOVLower = -15
gravity_ = 9.80665
def get_lidar_data(client, my_lidar_name, header):
    Lidar_data_raw = client.getLidarData(lidar_name=my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))
        num_temp = np.shape(points_position)[0]
        points_intensity = np.ones(num_temp)
        points_time = np.zeros(num_temp)
        points_color = np.ones((num_temp, 3)) * 255  # 设置默认颜色为白色
        points_alpha = np.ones(num_temp) * 255  # 设置默认透明度为不透明

        lidar_msg = PointCloud2()
        lidar_msg.height = 1
        lidar_msg.width = len(points_position)
        lidar_msg.header.frame_id = 'velodyne'
        lidar_msg.header.stamp = header.stamp
        lidar_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=12, datatype=PointField.UINT16, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgba", offset=24, datatype=PointField.UINT32, count=1)
        ]
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 28
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
        lidar_msg.is_dense = True

        points_ring = np.zeros(points_position.shape[0], dtype=int)

        ang_res = (VerticalFOVUpper - VerticalFOVLower) / float(NumberOfChannels)

        lidar_msg_data = []
        for i in range(points_position.shape[0]):
            x = points_position[i][0]
            y = points_position[i][1]
            z = points_position[i][2]
            verticalAngle = math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi
            points_ring[i] = (verticalAngle - VerticalFOVLower) // ang_res

            # 计算透明度
            alpha = calculate_alpha_based_on_some_condition()

            # 修改颜色和透明度信息
            rgba_color = (points_color[i][0] << 16) + (points_color[i][1] << 8) + points_color[i][2]
            rgba_color += (alpha << 24)

            lidar_msg_data.append([points_position[i][0], points_position[i][1], points_position[i][2],
                                   points_ring[i], points_intensity[i], points_time[i], rgba_color])

        lidar_msg.data = np.array(lidar_msg_data).astype(np.uint8).tobytes()
        pub_lidar.publish(lidar_msg)

def get_lidar_data(client,my_lidar_name, header):

    Lidar_data_raw = client.getLidarData(lidar_name = my_lidar_name)
    if len(Lidar_data_raw.point_cloud) > 3:
        points_position = np.array(Lidar_data_raw.point_cloud, dtype=np.dtype('f4'))
        points_position = np.reshape(points_position, (int(points_position.shape[0] / 3), 3))
        num_temp = np.shape(points_position)[0]
        points_intensity = np.ones(num_temp)
        points_time = np.zeros(num_temp)
        # # this is the rule in AIRSIM
        # delta_time = points_position.shape[0] / float(PointsPerSecond)
        # points_to_scan_with_one_laser = points_position.shape[0] // NumberOfChannels # this mush be an int


        lidar_msg = PointCloud2()
        lidar_msg.height = 1
        lidar_msg.width = len(points_position)
        lidar_msg.header.frame_id = 'velodyne'
        lidar_msg.header.stamp = header.stamp
        lidar_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=12, datatype=PointField.UINT16, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1)
            ]
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 24
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
        lidar_msg.is_dense = True


        points_ring = np.zeros(points_position.shape[0], dtype=int)
        
        ang_res = (VerticalFOVUpper - VerticalFOVLower) / float(NumberOfChannels)

        lidar_msg_data = []
        for i in range(points_position.shape[0]):
            x = points_position[i][0]
            y = points_position[i][1]
            z = points_position[i][2]
            verticalAngle = math.atan2(z, math.sqrt(x*x + y*y)) * 180 / math.pi
            points_ring[i] = (verticalAngle - VerticalFOVLower) // ang_res
            # lidar_msg_data.append([points_position[i][0], points_position[i][1], points_position[i][2], 
            #                        points_intensity[i], points_time[i], points_ring[i]])
            lidar_msg_data.append([points_position[i][0], points_position[i][1], points_position[i][2], 
                                    points_ring[i], points_intensity[i], points_time[i]])
        # 将点云数据转换为二进制字符串并赋值给消息对象
            

        # lidar_msg_data = np.c_[points_position, points_intensity, points_time]
        # lidar_msg_data = np.c_[points_position, points_intensity]

        lidar_msg.data = np.array(lidar_msg_data).astype(np.float32).tobytes()
        pub_lidar.publish(lidar_msg)


        return True
    else:
        return False

if __name__ == '__main__':
    try:

        rospy.init_node('airsim_test_slam_node', anonymous=True)
        rospy.loginfo('[airsim_test_slam_node] start')

        pub_lidar = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        pub_imu = rospy.Publisher('/imu/data', sensor_msgs.msg.Imu, queue_size=10)
        pub_imu_gt = rospy.Publisher('/imu/gt', sensor_msgs.msg.Imu, queue_size=10)
        pub_gps_fix = rospy.Publisher('/gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
        # pub_lidar = rospy.Publisher('points_raw', PointCloud2, queue_size=10)
        # pub_imu = rospy.Publisher('imu_raw', sensor_msgs.msg.Imu, queue_size=10)
        # pub_gps_fix = rospy.Publisher('/gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
        rate = rospy.Rate(400)

        client = airsim.MultirotorClient()
        client.confirmConnection()

        count_lidar = 0
        count_gps = 0
        while not rospy.is_shutdown():
            count_lidar = count_lidar + 1
            count_gps = count_gps + 1
            imu_msg = sensor_msgs.msg.Imu()
            imu_msg.header.frame_id = "world"
            imu_msg.header.stamp = rospy.Time.now()
            imu_data = client.getImuData(imu_name="Imu_mine")
            state_gt = client.simGetGroundTruthKinematics()
            gps_data = client.getGpsData()
            
            
            imu_msg.orientation.x = imu_data.orientation.x_val
            imu_msg.orientation.y = imu_data.orientation.y_val
            imu_msg.orientation.z = imu_data.orientation.z_val
            imu_msg.orientation.w = imu_data.orientation.w_val
            imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
            imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
            imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val

            imu_msg_gt = sensor_msgs.msg.Imu()
            imu_msg_gt.header.frame_id = "world"
            imu_msg_gt.header.stamp = rospy.Time.now()
            imu_msg_gt.orientation.x = state_gt.orientation.x_val
            imu_msg_gt.orientation.y = state_gt.orientation.y_val
            imu_msg_gt.orientation.z = state_gt.orientation.z_val
            imu_msg_gt.orientation.w = state_gt.orientation.w_val
            imu_msg_gt.angular_velocity.x = state_gt.angular_velocity.x_val
            imu_msg_gt.angular_velocity.y = state_gt.angular_velocity.y_val
            imu_msg_gt.angular_velocity.z = state_gt.angular_velocity.z_val
            imu_msg_gt.linear_acceleration.x = state_gt.linear_acceleration.x_val
            imu_msg_gt.linear_acceleration.y = state_gt.linear_acceleration.y_val
            imu_msg_gt.linear_acceleration.z = state_gt.linear_acceleration.z_val - gravity_

            pub_imu.publish(imu_msg)
            pub_imu_gt.publish(imu_msg_gt)

            # Publish lidar every 0.1 sec
            if count_lidar == 40:
                count_lidar = 0
                
                # if len(lidar_data_airsim.point_cloud) > 3:
                #     state = client.simGetGroundTruthKinematics()

                #     pcl_lidar = np.array(lidar_data_airsim.point_cloud, dtype=np.dtype('f4'))
                #     pcl_lidar = np.reshape(pcl_lidar, (int(pcl_lidar.shape[0] / 3), 3))
                #     # offset = np.array([state.position.x_val, state.position.y_val, state.position.z_val])
                #     # pcl_lidar += offset
                #     # pcl_lidar[:, 0] = np.round(pcl_lidar[:, 0]*2) / 2  # x坐标
                #     # pcl_lidar[:, 1] = np.round(pcl_lidar[:, 1]*2) / 2  # y坐标
                #     # pcl_lidar[:, 2] = np.round(pcl_lidar[:, 2]*2) / 2  # z坐标

                
                header_common = std_msgs.msg.Header()
                header_common.stamp = imu_msg.header.stamp
                header_common.frame_id = "world"
                    
                # get_lidar_data(client,'Lidar_test_slam', header_common)
                get_lidar_data(client,'Lidar_test_slam', header_common)

            if(count_gps == 100):
                count_gps = 0
                gps_msg = sensor_msgs.msg.NavSatFix()
                gps_msg.altitude = gps_data.gnss.geo_point.altitude
                gps_msg.longitude = gps_data.gnss.geo_point.longitude
                gps_msg.latitude = gps_data.gnss.geo_point.latitude
                gps_msg.header.stamp = imu_msg.header.stamp
                gps_msg.status.status = 2
                gps_msg.header.frame_id = "world"   # the frame of GPS should not be world
                pub_gps_fix.publish(gps_msg)


            rate.sleep()

    except rospy.ROSInterruptException:
        pass