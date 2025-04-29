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
import tf.transformations as tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import MessageInterval
from sensor_msgs.msg import Imu, CompressedImage, PointCloud2


first_flag = True
gazebo_init_position = [0, 0, 0]
gazebo_init_quaternion = [0, 0, 0, 1]
gazebo_init_euler = [0, 0, 0]
rotor_mode_evtol = 0
gazebo_rotor_mode_evtol = 0
count_num = 0
previous_yaw = 0
def normalize_yaw(yaw):
    """
    Normalize the yaw angle to be within the range [0, 2*pi).
    """
    return yaw % (2 * np.pi)
def gazebo_pose_cb4(data):
      # rospy.loginfo('get msg from gazebo')
        for i, model_name in enumerate(data.name):
            if model_name == 'standard_vtol':
                # rospy.loginfo('get pose from gazebo')
                global first_flag, gazebo_init_quaternion, gazebo_init_euler, gazebo_init_position
                if(first_flag):
                    pose_origin = data.pose[i]
                    rospy.logwarn('first in gazebo')
                    gazebo_init_position[0] = pose_origin.position.x
                    gazebo_init_position[1] = pose_origin.position.y
                    gazebo_init_position[2] = pose_origin.position.z
                    gazebo_init_quaternion[0] = pose_origin.orientation.x
                    gazebo_init_quaternion[1] = pose_origin.orientation.y
                    gazebo_init_quaternion[2] = pose_origin.orientation.z
                    gazebo_init_quaternion[3] = pose_origin.orientation.w
                    gazebo_init_euler = tf.euler_from_quaternion(gazebo_init_quaternion)
                    print(gazebo_init_euler)
                    print(gazebo_init_position)
                    first_flag = False
                    return
                pose_cur = data.pose[i]
                x_cur = x_origin + (pose_cur.position.y - gazebo_init_position[1])
                y_cur = y_origin + (pose_cur.position.x - gazebo_init_position[0]) # base on 
                z_cur = z_origin - (pose_cur.position.z - gazebo_init_position[2])

                position_airsim = airsim.Vector3r(x_cur, y_cur, z_cur)
                quaternion_cur = [pose_cur.orientation.x,
                                pose_cur.orientation.y,
                                pose_cur.orientation.z,
                                pose_cur.orientation.w]
                # relative_quat = tf.quaternion_multiply(quaternion_cur, tf.quaternion_inverse(gazebo_init_quaternion))
                # quaternion_cur = tf.quaternion_multiply(relative_quat, quaternion_origin)
                euler_cur = tf.euler_from_quaternion(quaternion_cur)
                euler_msg = Vector3()
                euler_msg.x = euler_cur[0]
                euler_msg.y = euler_cur[1]
                if (euler_cur[2]<-2.0):
                    euler_msg.z = normalize_yaw(euler_cur[2])
                else:
                    euler_msg.z = euler_cur[2]
                
                pub_euler.publish(euler_msg)

                roll_cur = euler_cur[0]
                pitch_cur = -euler_cur[1]
                yaw_cur = np.pi/2.0 - euler_cur[2]

                # print("in gazebo, roll = ", euler_cur[0], " pitch = ", euler_cur[1], " yaw = ", euler_cur[2])
            #  print("in gazebo, initial roll = ", gazebo_init_euler[0], " pitch = ", gazebo_init_euler[1], " yaw = ", gazebo_init_euler[2])
                # print("in airsim, roll = ", roll_cur[0], " pitch = ", pitch_cur[1], " yaw = ", yaw_cur[2])   
            #  print(data.pose[i])
                # print("x_cur = ", x_cur, " y_cur = ", y_cur, " z_cur = ", z_cur)

                # position_airsim = airsim.Vector3r(x_cur, -y_cur, -z_cur)

                quaternion_cur = tf.quaternion_from_euler(roll_cur, pitch_cur, yaw_cur)
                quaternion_airsim = airsim.Quaternionr(quaternion_cur[0],
                                                    quaternion_cur[1],
                                                    quaternion_cur[2],
                                                    quaternion_cur[3])
                # quaternion_airsim = airsim.Quaternionr(pose_cur.orientation.w,
                #                                     pose_cur.orientation.x,
                #                                     -pose_cur.orientation.y,
                #                                     -pose_cur.orientation.z)                
            #  quaternion_airsim = airsim.Quaternionr(quaternion_origin[0],
            #                                         quaternion_origin[1],
            #                                         quaternion_origin[2],
            #                                         quaternion_origin[3])
                client.simSetVehiclePose(airsim.Pose(position_airsim,quaternion_airsim),False)
                client.setRotorMode_evtol(rotor_mode_evtol)
                global count_num
                count_num = count_num + 1
                if(count_num>100):
                    count_num = 0
                    # print("in gazebo, x = ", pose_cur.position.x, " y = ", pose_cur.position.y, " z = ", pose_cur.position.z)
                    # print("in gazebo, roll = ", euler_cur[0], " pitch = ", euler_cur[1], " pitch = ", euler_cur[2])
                    # print("in airsim x_cur = ", x_cur, " y_cur = ", y_cur, " z_cur = ", z_cur)
                    # print("in airsim, roll = ", roll_cur, " pitch = ", euler_cur[1], " yaw = ", euler_cur[2])

def gazebo_link_cb(data):
    try:
        ZERO_THRESHOLD = 1
        # 查找四个常规转子的索引
        rotor_indices = [data.name.index("standard_vtol::rotor_" + str(i)) for i in range(4)]
        # 查找推进器转子的索引
        puller_index = data.name.index("standard_vtol::rotor_puller")
        
        # 判断lift电机是否都停止
        lift_stopped = all(abs(data.twist[index].angular.z) < ZERO_THRESHOLD for index in rotor_indices)

        # 判断推进器电机是否停止
        puller_stopped = abs(data.twist[puller_index].angular.z) < ZERO_THRESHOLD*0.1
        # puller_stopped = True
        # print("in gazebo, puller rotor speed is ", data.twist[puller_index].angular.z)

        global gazebo_rotor_mode_evtol, rotor_mode_evtol
        # 根据电机状态进行逻辑判断
        if lift_stopped and puller_stopped:
            gazebo_rotor_mode_evtol = 0
        elif lift_stopped and not puller_stopped:
            gazebo_rotor_mode_evtol = 3
        elif not lift_stopped and puller_stopped:
            gazebo_rotor_mode_evtol = 1
        else:
            gazebo_rotor_mode_evtol = 2

        if gazebo_rotor_mode_evtol != rotor_mode_evtol:
            rotor_mode_evtol = gazebo_rotor_mode_evtol
            # rospy.loginfo('change mode to {}'.format(rotor_mode_evtol))
            


    except ValueError:
        rospy.loginfo("One or more rotor links not found in the current message")

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
        # improve the imu frequency in gazebo
        set_message_interval()
        rospy.init_node('airsim_set_pose_node')
        rospy.loginfo('[airsim_set_pose_state_node] start')


        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)

        state = client.simGetGroundTruthKinematics()
        
        x_origin = state.position.x_val
        y_origin = state.position.y_val
        z_origin = state.position.z_val
        quaternion_origin = [state.orientation.x_val, state.orientation.y_val,
                             state.orientation.z_val, state.orientation.w_val]
        euler_origin = tf.euler_from_quaternion(quaternion_origin) # roll, pitch , yaw

        print("in airsim, initial position x = ", x_origin, " y = ", y_origin, " z = ", z_origin)
        print("in airsim, initial roll = ", euler_origin[0], " pitch = ", euler_origin[1], " yaw = ", euler_origin[2])
        client.setRotorMode_evtol(3)
        rospy.Subscriber('/gazebo/model_states',ModelStates,gazebo_pose_cb4)
        rospy.Subscriber('/gazebo/link_states',ModelStates,gazebo_link_cb)
        pub_euler = rospy.Publisher('/gazebo/euler_angles', Vector3, queue_size=10)
        # pub_airsim_pose = rospy.Publisher('/airsim/pose', Vector3, queue_size=10)
        # first_flag = True

        rospy.spin()


        rospy.loginfo('[airsim_set_pose_state_node] finish')




    except rospy.ROSInterruptException:
        pass   
    