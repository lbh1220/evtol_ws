#!/usr/bin/env python


import rospy
import airsim
import numpy as np
import time
import std_msgs.msg
import nav_msgs.msg
import os
import rospkg
import geometry_msgs.msg

origin_offset = [0, 0, 0]
resolution = 5.0
if __name__ == '__main__':
    try:
        
        rospy.init_node('airsim_pub_state_node')
        rospy.loginfo('[airsim_pub_state_node] start')

        origin_offset[0] = rospy.get_param("airsim/offset_x")
        origin_offset[1] = rospy.get_param("airsim/offset_y")
        origin_offset[2] = rospy.get_param("airsim/offset_z")
        resolution = rospy.get_param("airsim/resolution")


        client = airsim.MultirotorClient()
        pub_curr_state = rospy.Publisher('/airsim/aircraft/curr_state', nav_msgs.msg.Odometry, queue_size=10)
        pub_curr_state_com = rospy.Publisher('/airsim/aircraft/curr_state_com', nav_msgs.msg.Odometry, queue_size=10)
        pub_path_com = rospy.Publisher('/airsim/aircraft/path', nav_msgs.msg.Path, queue_size=10)

        rate = rospy.Rate(100)
        state = client.simGetGroundTruthKinematics()
        x_origin = state.position.x_val
        y_origin = state.position.y_val
        z_origin = state.position.z_val
        path_msg = nav_msgs.msg.Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = rospy.Time.now()
        
        path_count = 0
        while not rospy.is_shutdown():
            
            state = client.simGetGroundTruthKinematics()
            state_msg = nav_msgs.msg.Odometry()
            state_msg.pose.pose.position.x = state.position.x_val
            state_msg.pose.pose.position.y = state.position.y_val
            state_msg.pose.pose.position.z = state.position.z_val
            state_msg.pose.pose.orientation.x = state.orientation.x_val
            state_msg.pose.pose.orientation.y = state.orientation.y_val
            state_msg.pose.pose.orientation.z = state.orientation.z_val
            state_msg.pose.pose.orientation.w = state.orientation.w_val
            state_msg.twist.twist.linear.x = state.linear_velocity.x_val
            state_msg.twist.twist.linear.y = state.linear_velocity.y_val
            state_msg.twist.twist.linear.z = state.linear_velocity.z_val
            state_msg.twist.twist.angular.x = state.angular_velocity.x_val
            state_msg.twist.twist.angular.y = state.angular_velocity.y_val
            state_msg.twist.twist.angular.z = state.angular_velocity.z_val
            # state_msg.twist.twist.linear
            state_msg.header.frame_id = "world"
            state_msg.header.stamp = rospy.Time.now()
            pub_curr_state.publish(state_msg)


            state_msg_voxel = nav_msgs.msg.Odometry()
            state_msg_voxel.pose.pose.position.x =  -(state.position.y_val - y_origin)
            state_msg_voxel.pose.pose.position.y =  state.position.x_val - x_origin
            state_msg_voxel.pose.pose.position.z =  state.position.z_val - z_origin
            state_msg_voxel.pose.pose.orientation.x = state.orientation.x_val
            state_msg_voxel.pose.pose.orientation.y = state.orientation.y_val
            state_msg_voxel.pose.pose.orientation.z = state.orientation.z_val
            state_msg_voxel.pose.pose.orientation.w = state.orientation.w_val
            pose_msg = geometry_msgs.msg.PoseStamped()
            pose_msg.pose.position.x = state_msg_voxel.pose.pose.position.x
            pose_msg.pose.position.y = state_msg_voxel.pose.pose.position.y
            pose_msg.pose.position.z = state_msg_voxel.pose.pose.position.z
            pose_msg.pose.orientation.x = state.orientation.x_val
            pose_msg.pose.orientation.y = state.orientation.y_val
            pose_msg.pose.orientation.z = state.orientation.z_val
            pose_msg.pose.orientation.w = state.orientation.w_val
            pose_msg.header.frame_id = "world"
            
            state_msg_voxel.header.frame_id = "world"
            
            
            state_msg_voxel.header.stamp = rospy.Time.now()
            pose_msg.header.stamp = state_msg_voxel.header.stamp

            pub_curr_state_com.publish(state_msg_voxel)
            path_count = path_count + 1
            if(path_count == 10):
                path_count = 0
                path_msg.poses.append(pose_msg)
                pub_path_com.publish(path_msg)
            # TO DO, correct the orientation
            rate.sleep()




    except rospy.ROSInterruptException:
        pass   
    