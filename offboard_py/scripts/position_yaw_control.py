#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import math
import tf

def calculate_yaw(current_pos, target_pos):
    """计算从当前位置到目标位置的偏航角"""
    return math.atan2(target_pos.y - current_pos.y, target_pos.x - current_pos.x)



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
    rospy.init_node("position_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_target_pub = rospy.Publisher('/mavros/setpoint_raw/target_local', PositionTarget, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
        
    # Define the waypoints
    # waypoints = [
    #     [0, 0, 20],
    #     [30, 0, 20],
    #     [30, 30, 20],
    #     [0, 30, 20],
    #     [0, 0, 20]
    # ]
    waypoints = [
        [0, 0, 15],
        [40, 0, 15],
        [40, 40, 15],
        [0, 40, 15],
        [0, 0, 15]
    ]
    # Send a few setpoints before starting (to ensure the FCU is receiving them)
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
    threshold = 1

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            set_mode_client(custom_mode="OFFBOARD")
            last_req = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            arming_client(True)
            last_req = rospy.Time.now()

        if waypoint_index < len(waypoints) and distance_to_point(waypoints[waypoint_index]) < threshold:
            waypoint_index += 1  # Move to the next waypoint
            rospy.logwarn("arrive waypoints")

        if waypoint_index < len(waypoints):
            next_wp = waypoints[waypoint_index]

            target_waypoint = Point(next_wp[0], next_wp[1], next_wp[2])
            current_position = Point(current_pose.pose.position.x,
                                     current_pose.pose.position.y,
                                     current_pose.pose.position.z)
            yaw = calculate_yaw(current_position, target_waypoint)
            # 创建 PositionTarget 消息
            target = PositionTarget()
            target.header.stamp = rospy.Time.now()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
            target.position = target_waypoint
            target.yaw = yaw
        else:
            # After last waypoint, land at the origin
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 20
            local_pos_pub.publish(pose)
            # if distance_to_point([0, 0, 10]) < threshold:
            #     set_mode_client(custom_mode="AUTO.LAND")
            #     break
        rate.sleep()

