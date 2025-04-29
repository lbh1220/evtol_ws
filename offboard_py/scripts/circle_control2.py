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

def distance_to_point(wp):
    distance = ((current_pose.pose.position.x - wp[0]) ** 2 +
            (current_pose.pose.position.y - wp[1]) ** 2 +
            (current_pose.pose.position.z - wp[2]) ** 2) ** 0.5
    # rospy.loginfo("current distance is %s", distance)

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
    # Circle parameters
    center_x, center_y, altitude = -10, 0, 20
    radius = 10
    num_points = 50
    waypoints = []
    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)

        waypoints.append([x, y, altitude])
    waypoint_index = 0

    for i in range(100):
        if rospy.is_shutdown():
            break

        pose = PoseStamped()
        pose.pose.position.x = waypoints[waypoint_index][0]
        pose.pose.position.y = waypoints[waypoint_index][1]
        pose.pose.position.z = waypoints[waypoint_index][2]
        local_pos_pub.publish(pose)
        rate.sleep()

    last_req = rospy.Time.now()
    threshold = 0.5

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            set_mode_client(custom_mode="OFFBOARD")
            last_req = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            arming_client(True)
            last_req = rospy.Time.now()

        if waypoint_index < len(waypoints) and distance_to_point(waypoints[waypoint_index]) < threshold:
            waypoint_index += 1  # Move to the next waypoint
            # waypoint_index = waypoint_index // num_points
            rospy.logwarn("arrive waypoints")

        if waypoint_index < len(waypoints):
            pose = PoseStamped()
            pose.pose.position.x = waypoints[waypoint_index][0]
            pose.pose.position.y = waypoints[waypoint_index][1]
            pose.pose.position.z = waypoints[waypoint_index][2]
            local_pos_pub.publish(pose)
        else:
            # After last waypoint, land at the origin
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 10
            local_pos_pub.publish(pose)
            if distance_to_point([0, 0, 10]) < threshold:
                set_mode_client(custom_mode="AUTO.LAND")
                break
        rate.sleep()

