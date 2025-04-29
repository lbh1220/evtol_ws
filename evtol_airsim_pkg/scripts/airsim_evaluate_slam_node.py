#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from message_filters import TimeSynchronizer, Subscriber
from tf.transformations import euler_from_quaternion
import rospkg
import csv
import os
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import std_msgs.msg
import pcl_ros
time_slam = 0
time_gt = 0
def rcv_slam_callback(msg: Odometry):
    global time_slam
    time = msg.header.stamp.to_sec()
    if(time - time_slam)>0.1:
        time_slam = time
    else:
        return
    x_val = msg.pose.pose.position.x
    y_val = msg.pose.pose.position.y
    z_val = msg.pose.pose.position.z
    orientation_quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    )
    roll_val, pitch_val, yaw_val = euler_from_quaternion(orientation_quaternion)
    csv_writer_slam.writerow([time, x_val, y_val, z_val, yaw_val, roll_val, pitch_val])
    csv_file_slam.flush()
    rospy.loginfo("save slam")
    
def rcv_gt_callback(msg: Odometry):
    global time_gt
    time = msg.header.stamp.to_sec()
    if(time - time_gt)>0.1:
        time_gt = time
    else:
        return

    time = msg.header.stamp.to_sec()
    x_val = msg.pose.pose.position.x
    y_val = msg.pose.pose.position.y
    z_val = msg.pose.pose.position.z
    orientation_quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    )
    roll_val, pitch_val, yaw_val = euler_from_quaternion(orientation_quaternion)
    csv_writer_val.writerow([time, x_val, y_val, z_val, yaw_val, roll_val, pitch_val])
    csv_file_val.flush()
    rospy.loginfo("save ground truth")
    




def read_pcd_file(pcd_path):
    # 从.pcd文件中读取点云数据
    # 请自行实现读取.pcd文件的逻辑，并将点云数据转换为numpy数组形式
    # 这里只是一个示例
    # 读取.pcd文件
    pcd = o3d.io.read_point_cloud(pcd_path)

    # 将点云数据转换为numpy数组
    points = np.asarray(pcd.points)

    return points
if __name__ == '__main__':
    try:
        count1 = 0
        count2 = 0  
        rospy.init_node('airsim_evaluate_slam_node')
        rospy.loginfo("airsim_evaluate_slam_node start")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('evtol_airsim_pkg')
        csv_filename_val = os.path.join(pkg_path, 'maps', 'odom_gt.csv')
        csv_file_val = open(csv_filename_val, 'w')
        csv_writer_val = csv.writer(csv_file_val)
        csv_writer_val.writerow(['Timestamp', 'GT X', 'GT Y', 'GT Z', 'GT yaw', 'GT roll', 'GT pitch'])
        csv_filename_slam = os.path.join(pkg_path, 'maps', 'odom_slam.csv')
        csv_file_slam = open(csv_filename_slam, 'w')
        csv_writer_slam = csv.writer(csv_file_slam)
        csv_writer_slam.writerow(['Timestamp', 'GT X', 'GT Y', 'GT Z', 'GT yaw', 'GT roll', 'GT pitch'])

        rospy.Subscriber('/mavros/local_position/odom', Odometry, rcv_gt_callback)
        rospy.Subscriber('/Odometry', Odometry, rcv_slam_callback)
        rospy.spin()
    #     map_pub = rospy.Publisher('pcd_map', PointCloud2, queue_size=10)
    #             # 读取.pcd文件
    #     pcd_path = os.path.join(pkg_path, 'maps', 'scans.pcd')
    #     rospy.loginfo("start read")
    #     point_cloud = read_pcd_file(pcd_path)
    #     rospy.loginfo("finish read")
    #     # 定义消息头
    #     header = std_msgs.msg.Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "map"

    #     # 创建PointCloud2消息
    #     pcd_msg = point_cloud2.create_cloud_xyz32(header, point_cloud)

    # # 发布PointCloud2消息
    #     rate = rospy.Rate(1)  # 设置发布频率为1Hz
    #     while not rospy.is_shutdown():
    #         map_pub.publish(pcd_msg)
    #         rospy.loginfo("publish map")
    #         rate.sleep()        # rospy.spin()
        


    except rospy.ROSInterruptException:
        pass
