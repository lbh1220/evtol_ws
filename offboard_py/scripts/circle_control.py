#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg
def state_cb(msg):
    global current_state
    current_state = msg



def distance_to_point(wp):
    distance = ((current_pose.pose.position.x - wp[0]) ** 2 +
            (current_pose.pose.position.y - wp[1]) ** 2 +
            (current_pose.pose.position.z - wp[2]) ** 2) ** 0.5
    rospy.loginfo("current distance is %s", distance)
    return distance

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    local_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

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
    pose.pose.position.z = 20

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    flag_start_circle = False

    center = np.array([0, 5])  # Circle center relative to starting point
    radius = 5
    speed = 2.0
    clock_wise = True  # 顺时针或逆时针设置
    rate = rospy.Rate(50)
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        if flag_start_circle:
            current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
            direction_to_center = current_position - center
            distance_to_center = np.linalg.norm(direction_to_center)
            if (distance_to_center - radius) > 0.1:
                vel_dir_1 = -direction_to_center
            else:
                vel_dir_1 = direction_to_center
            theta = math.atan2(direction_to_center[1], direction_to_center[0])
            if clock_wise:
                theta += math.pi / 2
            else:
                theta -= math.pi / 2
            v_dir_2 = np.array([math.cos(theta), math.sin(theta)])
            v_dir = 0.08 * vel_dir_1 + v_dir_2
            v_cmd = speed * v_dir / np.linalg.norm(v_dir)
            print(v_cmd)

            vel_msg = TwistStamped()
            vel_msg.twist.linear.x = v_cmd[0]
            vel_msg.twist.linear.y = v_cmd[1]
            vel_msg.twist.linear.z = 0
            rospy.loginfo("speed cmd x = %s, y= %s", v_cmd[0], v_cmd[1])
            velocity_pub.publish(vel_msg)
            rate.sleep()
        else:
            local_pos_pub.publish(pose)
            if( current_pose.pose.position.z > 20):
                flag_start_circle = True
                rospy.logwarn("arrive target height")
        

        rate.sleep()