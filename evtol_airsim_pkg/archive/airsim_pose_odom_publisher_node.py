#!/usr/bin/env python

import rospy
import airsim
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf.transformations

class AirSimPoseOdomPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('airsim_pose_odom_publisher')
        rospy.loginfo("airsim_pose_odom_publisher node initialized")
        
        # 创建AirSim客户端
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        
        # 创建发布器
        self.pose_pub = rospy.Publisher('/airsim/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/airsim/odom', Odometry, queue_size=10)
        
        # # NED到ENU的转换矩阵
        # self.transformation_ned_to_enu = np.array([
        #     [1, 0, 0, 0],  
        #     [0, -1, 0, 0],   
        #     [0, 0, -1, 0],  
        #     [0, 0, 0, 1] 
        # ])
        self.transformation_ned_to_enu = np.array([
            [0, 1, 0, 0],  
            [1, 0, 0, 0],   
            [0, 0, -1, 0],  
            [0, 0, 0, 1] 
        ])
        # 创建定时器
        rospy.Timer(rospy.Duration(0.01), self.pose_callback)  # 100Hz

    def transform_point_ned_to_enu(self, point_ned):
        """将位置从NED转换到ENU坐标系"""
        point_homogeneous = np.array([point_ned.x_val, point_ned.y_val, point_ned.z_val, 1.0])
        point_transformed = np.dot(self.transformation_ned_to_enu, point_homogeneous)
        return Point(
            x=point_transformed[0],
            y=point_transformed[1],
            z=point_transformed[2]
        )

    def transform_vector_ned_to_enu(self, vector_ned):
        """将向量（速度、角速度）从NED转换到ENU坐标系"""
        vector_homogeneous = np.array([vector_ned.x_val, vector_ned.y_val, vector_ned.z_val, 0.0])  # 注意这里用0而不是1
        vector_transformed = np.dot(self.transformation_ned_to_enu, vector_homogeneous)
        return Vector3(
            x=vector_transformed[0],
            y=vector_transformed[1],
            z=vector_transformed[2]
        )

    # def transform_orientation_ned_to_enu(self, q_ned):
    #     """将四元数从NED转换到ENU坐标系"""
    #     # 首先将四元数转换为欧拉角
    #     euler = tf.transformations.euler_from_quaternion([q_ned.x_val, q_ned.y_val, q_ned.z_val, q_ned.w_val])
        
    #     # 调整欧拉角以适应ENU坐标系
    #     # roll保持不变，pitch和yaw需要取反
    #     euler_enu = (euler[0], -euler[1], -euler[2])
        
    #     # 将调整后的欧拉角转回四元数
    #     q_enu = tf.transformations.quaternion_from_euler(*euler_enu)
    #     rospy.loginfo(f"q_enu: w={q_enu[3]}, x={q_enu[0]}, y={q_enu[1]}, z={q_enu[2]}")   
    #     rospy.loginfo(f"q_ned: w={q_ned.w_val}, x={q_ned.x_val}, y={q_ned.y_val}, z={q_ned.z_val}")
        
    #     return Quaternion(x=q_enu[0], y=q_enu[1], z=q_enu[2], w=q_enu[3])
    def transform_orientation_ned_to_enu(self, q_ned):
        """将四元数从NED转换到ENU坐标系"""
        # 首先将四元数转换为欧拉角
        euler = tf.transformations.euler_from_quaternion([q_ned.x_val, q_ned.y_val, q_ned.z_val, q_ned.w_val])
        
        # 调整欧拉角以适应ENU坐标系
        # roll保持不变，pitch和yaw需要取反
        euler_enu = (euler[0], -euler[1], np.pi/2.0-euler[2])
        
        # 将调整后的欧拉角转回四元数
        q_enu = tf.transformations.quaternion_from_euler(*euler_enu)
        rospy.loginfo(f"q_enu: w={q_enu[3]}, x={q_enu[0]}, y={q_enu[1]}, z={q_enu[2]}")   
        rospy.loginfo(f"q_ned: w={q_ned.w_val}, x={q_ned.x_val}, y={q_ned.y_val}, z={q_ned.z_val}")
        
        return Quaternion(x=q_enu[0], y=q_enu[1], z=q_enu[2], w=q_enu[3])
    def pose_callback(self, event):
        """发布位姿和里程计数据"""
        try:
            # 获取地面真值运动学数据
            state = self.client.simGetGroundTruthKinematics()
            
            # 获取当前时间戳
            current_time = rospy.Time.now()
            
            # 转换位置和姿态到ENU坐标系
            position_enu = self.transform_point_ned_to_enu(state.position)
            orientation_enu = self.transform_orientation_ned_to_enu(state.orientation)
            
            # 转换线速度和角速度到ENU坐标系
            linear_velocity_enu = self.transform_vector_ned_to_enu(state.linear_velocity)
            angular_velocity_enu = self.transform_vector_ned_to_enu(state.angular_velocity)
            
            # 创建PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position = position_enu
            pose_msg.pose.orientation = orientation_enu
            
            # 创建Odometry消息
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose = pose_msg.pose
            
            # 设置转换后的速度
            twist = Twist()
            twist.linear = linear_velocity_enu
            twist.angular = angular_velocity_enu
            odom_msg.twist.twist = twist
            
            # 发布消息
            self.pose_pub.publish(pose_msg)
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in pose publisher: {str(e)}")

    def run(self):
        """运行发布器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = AirSimPoseOdomPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 