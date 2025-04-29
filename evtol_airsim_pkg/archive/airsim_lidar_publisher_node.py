#!/usr/bin/env python

import rospy
import numpy as np
import airsim
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import time

class AirSimLidarPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('airsim_lidar_publisher')
        rospy.loginfo("airsim_lidar_publisher node initialized")
        
        # 创建AirSim客户端
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        
        # 创建发布器
        self.lidar_pub = rospy.Publisher('/airsim/point_cloud', PointCloud2, queue_size=10)
        
        # 添加锁标志
        self.is_processing_lidar = False
        
        # NED到ENU的转换矩阵
        self.transformation_ned_to_enu = np.array([
            [1, 0, 0, 0],  
            [0, -1, 0, 0],   
            [0, 0, -1, 0],  
            [0, 0, 0, 1] 
        ])
        
        # 创建定时器
        rospy.Timer(rospy.Duration(0.2), self.lidar_callback)  # 5Hz

    def transform_points_ned_to_enu(self, points):
        """将点云从NED坐标系转换到ENU坐标系"""
        # 将点云数据转换为齐次坐标
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        
        # 应用变换
        points_transformed = np.dot(points_homogeneous, self.transformation_ned_to_enu.T)
        
        # 返回转换后的3D点
        return points_transformed[:, :3]

    def lidar_callback(self, event):
        """发布激光雷达点云数据"""
        if self.is_processing_lidar:
            return
            
        try:
            self.is_processing_lidar = True
            
            # 获取激光雷达数据
            lidar_data = self.client.getLidarData(lidar_name="Lidar_Avia_front")
            
            if (len(lidar_data.point_cloud) >= 3) and (len(lidar_data.point_cloud) % 3 == 0):
                # 将点云数据转换为numpy数组
                points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
                
                # 将点云从NED转换到ENU坐标系
                points_enu = self.transform_points_ned_to_enu(points)
                
                # 创建PointCloud2消息
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "map"
                
                # 创建并发布点云消息
                cloud_msg = create_cloud_xyz32(header, points_enu)
                self.lidar_pub.publish(cloud_msg)
                
            time.sleep(0.01)  # 添加小延迟确保数据处理完成
            
        except Exception as e:
            rospy.logerr(f"Error in lidar publisher: {str(e)}")
        finally:
            self.is_processing_lidar = False

    def run(self):
        """运行发布器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = AirSimLidarPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 