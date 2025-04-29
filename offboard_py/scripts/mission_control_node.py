#!/usr/bin/env python

import rospy
import math
from math import atan2, pi
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandVtolTransition, CommandVtolTransitionRequest, CommandTOL
from gazebo_msgs.msg import ModelStates
import tf.transformations as tf
from geometry_msgs.msg import Vector3


current_state = State()
current_pose = PoseStamped()
current_pose_gazebo = PoseStamped()
current_vel = TwistStamped()
def state_cb(msg):
    global current_state
    current_state = msg
    # rospy.loginfo("get state")
def pose_cb(msg):
    global current_pose
    current_pose = msg
    quaternion_cur = [current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w]
    euler_cur = tf.euler_from_quaternion(quaternion_cur)
    euler_msg = Vector3()
    euler_msg.x = euler_cur[0]
    euler_msg.y = euler_cur[1]
    euler_msg.z = euler_cur[2]
    
    # pub_euler.publish(euler_msg)
def vel_cb(msg):
    global current_vel
    current_vel = msg
    speed_cur = current_vel.twist.linear.x * current_vel.twist.linear.x
    speed_cur = speed_cur + current_vel.twist.linear.y * current_vel.twist.linear.y
    speed_cur = math.sqrt(speed_cur)
    # rospy.logwarn("current lateral speed : %s", speed_cur)    
def gazebo_pose_cb(data):
      # rospy.loginfo('get msg from gazebo')
      for i, model_name in enumerate(data.name):
            if model_name == 'standard_vtol':
                pose_gazebo = data.pose[i]
                global current_pose_gazebo
                current_pose_gazebo.pose.position.x = pose_gazebo.position.x
                current_pose_gazebo.pose.position.y = pose_gazebo.position.y
                current_pose_gazebo.pose.position.z = pose_gazebo.position.z
def vtol_transtion(target_state):

    request = CommandVtolTransitionRequest()
    request.state = target_state
    #   // If the parameter is set to 4 then it will change to fixed wing mode
    #   // If the parameter is set to 3 then the aircraft will change to VTOL mode
    flag = False
    global current_state
    # rospy.logwarn("Waiting for vehicle to be armed and in OFFBOARD mode...")
    while not rospy.is_shutdown():
        if current_state.armed and current_state.mode == "OFFBOARD":
            try:
                response = transition_client(request)
                if response.success:
                    rospy.logwarn("VTOL transition successful!")
                    flag = True
                    return flag
                else:
                    rospy.logwarn("VTOL transition failed with response: %s", response)
                    flag = False
                    return flag
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
                flag = False
                return flag
    # rospy.sleep(1)

def distance_to_point(wp, dim = 2):

    if dim == 2:
        distance = ((current_pose.pose.position.x - wp[0]) ** 2 +
                (current_pose.pose.position.y - wp[1]) ** 2) ** 0.5
    elif dim == 3:
        distance = ((current_pose.pose.position.x - wp[0]) ** 2 +
                (current_pose.pose.position.y - wp[1]) ** 2 +
                (current_pose.pose.position.z - wp[2]) ** 2) ** 0.5

    return distance

def normalize_angle(angle):
    """ Normalize angle to be within the range [-pi, pi] """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def calculate_yaw_difference(target_yaw, current_yaw):
    """ Calculate the smallest difference between two yaw angles """
    difference = normalize_angle(target_yaw - current_yaw)
    return difference

if __name__ == "__main__":
    rospy.init_node("mission_control_node")
    
    # ***********Subscribe from gazebo and mavros******
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = pose_cb)
    vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)
    
    rospy.Subscriber('/gazebo/model_states',ModelStates,gazebo_pose_cb)
    # *************************************************


    # **********Publish to mavros********
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    # ***********************************
    

    # **********Mavros service************
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)


    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/vtol_transition")
    transition_client = rospy.ServiceProxy("/mavros/cmd/vtol_transition", CommandVtolTransition)

    rospy.wait_for_service("/mavros/cmd/land")
    land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
    # ************************************


    # *******Publish to other node******
    # pub_euler = rospy.Publisher('/gazebo/euler_angles', Vector3, queue_size=10)
    # **********************************


    # *********flight parameter*********
    takeoff_height = 20.0
    cruise_height = 30.0
    landing_height = 20.0


    waypoints = [
        [0, 0, 50],
        [100, 0, 50],
        [500, 0, 50],
        [750, 0, 50],
        [750, 150, 50],
        [500, 150, 50],
        [500, 0, 50],
        [300, 0, 30],
        [0, 0, 30]
    ]
    # waypoints = [
    #     [0, 0, 50],
    #     [0, 100, 50],
    #     [0, 500, 50],
    #     [0, 750, 50],
    #     [150, 750, 50],
    #     [150, 500, 50],
    #     [0, 500, 50],
    #     [0, 300, 30],
    #     [0, 0, 30]
    # ]
    # waypoints = [
    #     [0, 0, 20],
    #     [0, 100, 20],
    #     [100, 100, 20],
    #     [100, 0, 20],
    #     [0, 0, 20]
    # ]
    distance_threshold_MR = 3
    distance_threshold_FW = 50
    distance_trans_back = 120
    distance_trans_forward = 0.5

    transition_FW_waypoint_idx = 0

    transition_MR_waypoint_idx = len(waypoints) - 1

    landing_waypoint_idx = len(waypoints) - 1

    changing_yaw = True
    yaw_threshold = 10.0*pi/180.0

    #***********************************


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    vel_target = Twist()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = takeoff_height

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
    flag_takeoff = False
    flag_transition = False
    flag_transback = False

    mode_list = ['offboard', 'takeoff', 'MR', 'transition2FW', 'FW', 'transition2MR', 'landing']
    curr_mode = 'takeoff'
    curr_waypoint_idx = 0
    while(not rospy.is_shutdown()):
        # 起飞前的准备
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.logwarn("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.logwarn("Vehicle armed")
                last_req = rospy.Time.now()

        # 加载当前的航路点
        if curr_waypoint_idx < len(waypoints):
            target_waypoint = waypoints[curr_waypoint_idx]
        else:
            curr_waypoint_idx = len(waypoints) - 1
            target_waypoint = waypoints[curr_waypoint_idx]

        # 起飞
        if curr_mode == 'takeoff':
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = takeoff_height
            local_pos_pub.publish(pose)
            if(abs(current_pose.pose.position.z - takeoff_height) < 1):
                rospy.logwarn("Takeoff: finish takeoff")
                # switch mode to MR
                curr_mode = 'MR'

        # multi rotor模态
        elif curr_mode == 'MR':
           
            pose.pose.position.x = target_waypoint[0]
            pose.pose.position.y = target_waypoint[1]
            pose.pose.position.z = target_waypoint[2]

            if changing_yaw and curr_waypoint_idx > 0:
                prev_waypoint = waypoints[curr_waypoint_idx - 1]
                dx = target_waypoint[0] - prev_waypoint[0]
                dy = target_waypoint[1] - prev_waypoint[1]
                yaw = math.atan2(dy, dx)  # Calculate yaw to face towards the next waypoint
                target_quat = tf.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = target_quat[0]
                pose.pose.orientation.y = target_quat[1]
                pose.pose.orientation.z = target_quat[2]
                pose.pose.orientation.w = target_quat[3]
            local_pos_pub.publish(pose)
            
            # 判断是否到达下一个航路点
            if distance_to_point(target_waypoint, dim=3) < distance_threshold_MR:
                if curr_waypoint_idx < len(waypoints):
                    rospy.logwarn(f"MR: arrive waypoint {curr_waypoint_idx}")
                    curr_waypoint_idx += 1

            # 这个判断的顺序有要求的, 先判断更大一点的idx
            if curr_waypoint_idx > landing_waypoint_idx:
                curr_mode = 'landing'
                rospy.logwarn("MR: prepare landing")
            elif curr_waypoint_idx > transition_FW_waypoint_idx:
                if not flag_transback:
                    # 增加flag_transback的判断, transition回来准备降落的时候, 如果尚未到达landing point, 这里会再次transition2FW
                    # TODO Adjust yaw before transitioning to fixed-wing mode
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                    curr_mode = 'transition2FW'
                    rospy.logwarn("MR: prepare transition to FW")

        # fix wing 模态
        elif curr_mode == 'FW':

            pose.pose.position.x = target_waypoint[0]
            pose.pose.position.y = target_waypoint[1]
            pose.pose.position.z = target_waypoint[2]
            local_pos_pub.publish(pose)
            
            # 判断是否到达下一个航路点
            if distance_to_point(target_waypoint, dim=3) < distance_threshold_FW:
                if curr_waypoint_idx < len(waypoints):
                    rospy.logwarn(f"FW: arrive waypoint {curr_waypoint_idx}")
                    curr_waypoint_idx += 1
            elif curr_waypoint_idx == transition_MR_waypoint_idx:
                if distance_to_point(target_waypoint, dim=3) < distance_trans_back:
                    curr_mode = 'transition2MR'
                    rospy.logwarn("FW: prepare transition to MR")

            # 判断是否到达预定的transition point
            if curr_waypoint_idx > transition_MR_waypoint_idx:
                curr_mode = 'transition2MR'
                rospy.logwarn("FW: prepare transition to MR")

        # multi rotor向fix wing做transition
        elif curr_mode == 'transition2FW':
            prev_waypoint = waypoints[curr_waypoint_idx - 1]
            dx = target_waypoint[0] - prev_waypoint[0]
            dy = target_waypoint[1] - prev_waypoint[1]
            target_yaw = math.atan2(dy, dx)  # Calculate yaw to face towards the next waypoint
            quaternion_cur = [current_pose.pose.orientation.x,
                            current_pose.pose.orientation.y,
                            current_pose.pose.orientation.z,
                            current_pose.pose.orientation.w]
            euler_cur = tf.euler_from_quaternion(quaternion_cur)
            current_yaw = euler_cur[2]
            yaw_difference = calculate_yaw_difference(target_yaw, current_yaw)
            finish_yaw = abs(yaw_difference)<yaw_threshold
            finish_position = distance_to_point(prev_waypoint, dim=3) < distance_trans_forward

            if finish_yaw and finish_position:
                rospy.logwarn("transition2FW: finish yaw adjust, start transition")
                flag_transition = vtol_transtion(4)
                while(not rospy.is_shutdown()):
                    speed_cur = current_vel.twist.linear.x * current_vel.twist.linear.x
                    speed_cur = speed_cur + current_vel.twist.linear.y * current_vel.twist.linear.y
                    speed_cur = math.sqrt(speed_cur)
                    if(speed_cur >=1.0):
                        curr_mode = 'FW'
                        rospy.logwarn("transition2FW: transition to fixwing mode")
                        break
                    rate.sleep()
            else:
                target_quat = tf.quaternion_from_euler(0, 0, target_yaw)
                pose.pose.orientation.x = target_quat[0]
                pose.pose.orientation.y = target_quat[1]
                pose.pose.orientation.z = target_quat[2]
                pose.pose.orientation.w = target_quat[3]
                # 应该让他保持在上一个waypoint不要动才行
                pose.pose.position.x = prev_waypoint[0]
                pose.pose.position.y = prev_waypoint[1]
                pose.pose.position.z = prev_waypoint[2]
                local_pos_pub.publish(pose)


        # fix wing向multi rotor做transition
        elif curr_mode == 'transition2MR':
            flag_transback = vtol_transtion(3)
            if flag_transback:
                # 按理说应该是MR的状态, 按照轨迹飞到预定降落点才能切换为landing模式
                rospy.logwarn("transition2MR: transition to multi-rotor mode")
                curr_mode = 'MR'

        elif curr_mode == 'landing':

            landing_position = [target_waypoint[0],
                                target_waypoint[1],
                                landing_height]
            pose.pose.position.x = landing_position[0]
            pose.pose.position.y = landing_position[1]
            pose.pose.position.z = landing_position[2]
            # Calculate the yaw from the previous to the last waypoint
            if changing_yaw and curr_waypoint_idx > 0:
                prev_waypoint = waypoints[curr_waypoint_idx - 1]
                dx = landing_position[0] - prev_waypoint[0]
                dy = landing_position[1] - prev_waypoint[1]
                yaw = math.atan2(dy, dx)  # Calculate yaw to face towards the next waypoint
                target_quat = tf.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = target_quat[0]
                pose.pose.orientation.y = target_quat[1]
                pose.pose.orientation.z = target_quat[2]
                pose.pose.orientation.w = target_quat[3]
            if distance_to_point(target_waypoint, dim=3) < distance_threshold_MR:
                try:
                    # 0, 0, 0, 0 can be replaced with actual coordinates if required
                    response = land_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=0)
                    if response.success:
                        rospy.loginfo("Landing initiated")
                        break
                    else:
                        rospy.loginfo("Landing command failed")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
            else:
                local_pos_pub.publish(pose)

        rate.sleep()