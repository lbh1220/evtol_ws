#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from nav_msgs.msg import Path, Odometry

# from octomap_msgs.srv import BinaryMapToMsg, BinaryMapToMsgRequest
import octomap
import tf.transformations
import rospkg
import os

class LidarMapping:
    def __init__(self):
        rospy.init_node('lidar_mapping_node', anonymous=True)
        self.map_pub = rospy.Publisher('/octomap_full', Octomap, queue_size=10)
        self.lidar_sub_front = rospy.Subscriber('/airsim/lidar/point_cloud/Lidar_Avia_front', PointCloud2, self.lidar_callback, callback_args='Lidar_Avia_front')
        self.lidar_sub_bottom = rospy.Subscriber('/airsim/lidar/point_cloud/Lidar_mid360_bottom', PointCloud2, self.lidar_callback, callback_args='Lidar_mid360_bottom')
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.tfd_lidar_pub_front = rospy.Publisher('/map/Lidar_Avia_front', PointCloud2, queue_size=10)
        self.tfd_lidar_pub_bottom = rospy.Publisher('/map/Lidar_mid360_bottom', PointCloud2, queue_size=10)
        self.current_pose = None
        self.octree = octomap.OcTree(1)  # Resolution of the Octomap
        self.path_pub = rospy.Publisher('/airsim/trajectory', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "map"  # 确保这个frame_id与你的TF树相匹配

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg

    def lidar_callback(self, cloud_msg, topic_name):
        if self.current_pose is not None:
            points = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)))
            transformed_points = self.transform_points(points)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"

            lidar_msg = pc2.create_cloud_xyz32(header, transformed_points)
            if(topic_name == 'Lidar_mid360_bottom'):
                self.tfd_lidar_pub_bottom.publish(lidar_msg)
            elif(topic_name == 'Lidar_Avia_front'):
                self.tfd_lidar_pub_front.publish(lidar_msg)
            # self.update_map(transformed_points)
            self.update_path(self.current_pose)

    def transform_points(self, points):
        # Extract translation
        translation = np.array([self.current_pose.pose.position.x,
                                self.current_pose.pose.position.y,
                                self.current_pose.pose.position.z])

        # Extract rotation quaternion
        quaternion = [self.current_pose.pose.orientation.x,
                      self.current_pose.pose.orientation.y,
                      self.current_pose.pose.orientation.z,
                      self.current_pose.pose.orientation.w]
        
        # Create transformation matrix
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
        transformation_matrix[:3, 3] = translation

        # Apply transformation
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points_homogeneous = np.dot(transformation_matrix, points_homogeneous.T).T
        
        return transformed_points_homogeneous[:, :3]

    def update_path(self, pose):
        # 添加当前位置到路径
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose.pose
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_stamped)

        # 发布路径
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def update_map(self, points):
        # 确保points是一个二维数组且数据类型为float64
        if points.ndim == 1:
            points = points.reshape(-1, 3)
        points = points.astype(np.float64)  # 转换数据类型

        origin = np.array([0, 0, 0], dtype=np.float64)  # 设置原点也为float64类型
        self.octree.insertPointCloud(points, origin)

        # 创建并发布Octomap消息
        # octomap_msg = fullMapToMsg(self.octree)
        # self.map_pub.publish(octomap_msg)

    def run(self):
        rate = rospy.Rate(2)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()
    def save_octomap(self, filename):
        if self.octree:
            # Ensure the filename is in bytes if the API requires it
            bytes_filename = filename.encode('utf-8')
            self.octree.writeBinary(bytes_filename)
            print("Octomap has been saved to", filename)
        else:
            print("No octree data to save.")

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('evtol_airsim_pkg')
    map_filename = os.path.join(pkg_path, 'maps', 'octomap.bt')
    try:
        mapper = LidarMapping()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pass
        # 程序准备结束，保存地图
        # mapper.save_octomap(map_filename)