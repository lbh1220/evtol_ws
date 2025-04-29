#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget, Thrust
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import Quaternion
current_state = State()
current_pose = PoseStamped()
current_vel = TwistStamped()
def state_cb(msg):
    global current_state
    current_state = msg
def pose_cb(msg):
    global current_pose
    current_pose = msg
def vel_cb(msg):
    global current_vel
    current_vel = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    thrust_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = pose_cb)
    vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    thrust_msg = AttitudeTarget()
    thrust_msg.type_mask = 7
    # thrust_msg.header.
    # thrust_msg.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | \
    #                 AttitudeTarget.IGNORE_PITCH_RATE | \
    #                 AttitudeTarget.IGNORE_YAW_RATE | \
    #                 AttitudeTarget.IGNORE_ATTITUDE
    thrust_msg.orientation = Quaternion(0, 0, 0, 1) 
    thrust_msg.thrust = 0.6  # 设置推力值，范围通常是0到1
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        thrust_pub.publish(thrust_msg)
        # local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    flag_arm = False
    flag_state = 0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    flag_arm = True

                last_req = rospy.Time.now()
        # rospy.loginfo("current position in gazebo: x = %s, y = %s, z = %s", 
        #               current_pose.pose.position.x,
        #               current_pose.pose.position.y,
        #               current_pose.pose.position.z)
        # rospy.loginfo("current speed in gazebo: x = %s, y = %s, z = %s", 
        #         current_vel.twist.linear.x,
        #         current_vel.twist.linear.y,
        #         current_vel.twist.linear.z)
        if flag_state == 0:
            if current_pose.pose.position.z > 10:
                thrust_msg.thrust = 0.526
                flag_state = 1
                rospy.loginfo("finish climb, arrive 10m")
        elif flag_state == 1:
            if current_vel.twist.linear.z < 0.01:
                thrust_msg.thrust = 0.527
                flag_state = 2
                rospy.loginfo("finish deacc")
        else:
            # thrust_msg.thrust = 0.5
            pass

        thrust_pub.publish(thrust_msg)

        rate.sleep()