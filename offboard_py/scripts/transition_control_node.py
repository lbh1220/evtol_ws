#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandVtolTransition, CommandVtolTransitionRequest
from gazebo_msgs.msg import ModelStates
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
    rospy.logwarn("Waiting for vehicle to be armed and in OFFBOARD mode...")
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
    rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = pose_cb)
    vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)
    
    rospy.Subscriber('/gazebo/model_states',ModelStates,gazebo_pose_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)


    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/vtol_transition")
    transition_client = rospy.ServiceProxy("/mavros/cmd/vtol_transition", CommandVtolTransition)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    vel_target = Twist()

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
    flag_takeoff = False
    flag_transition = False
    flag_transback = False

    mode_list = ['offboard', 'takeoff', 'MR', 'transition2FW', 'FW', 'transition2MR', 'landing']
    curr_mode = 'offboard'
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                curr_mode = 'takeoff'

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        
        # local_pos_pub.publish(pose)
        # rospy.loginfo("current position: x = %s, y = %s, z = %s", 
        #               current_pose.pose.position.x, 
        #               current_pose.pose.position.y,
        #               current_pose.pose.position.z)
        rospy.loginfo("current position in gazebo: x = %s, y = %s, z = %s", 
                      current_pose_gazebo.pose.position.x,
                      current_pose_gazebo.pose.position.y,
                      current_pose_gazebo.pose.position.z)
        if(current_pose.pose.position.z > 20):
            flag_takeoff = True
            if(not flag_takeoff):
                rospy.loginfo("takeoff finished")
                

            if(not flag_transition):
                flag_transition = vtol_transtion(4)
                # rospy.sleep(0.3)
                # flag_transition = vtol_transtion()
                # after transition, do not set waypoint immediately !!!
                while(not rospy.is_shutdown()):
                    speed_cur = current_vel.twist.linear.x * current_vel.twist.linear.x
                    speed_cur = speed_cur + current_vel.twist.linear.y * current_vel.twist.linear.y
                    speed_cur = math.sqrt(speed_cur)
                    # rospy.logwarn("current lateral speed : %s", speed_cur)
                    # pose.pose.position.x = current_pose.pose.position.x + 20
                    # pose.pose.position.y = 0
                    # pose.pose.position.z = 30
                    # local_pos_pub.publish(pose)
                    # vel_target.linear.x = current_vel.twist.linear.x + 2
                    # vel_target.linear.y = 0
                    # vel_target.linear.z = 0
                    # vel_target.angular.x = 0
                    # vel_target.angular.y = 0
                    # vel_target.angular.z = 0
                    # local_vel_pub.publish(vel_target)
                    if(speed_cur >=2.0):
                        break
                    rate.sleep()
            # rospy.loginfo("finished transition")
            if(current_pose_gazebo.pose.position.x<200):
                pose.pose.position.x = current_pose.pose.position.x + 200
                pose.pose.position.y = 0
                pose.pose.position.z = 20
                local_pos_pub.publish(pose)
            else:
                if(not flag_transback):
                    flag_transback = vtol_transtion(3)
                # while(not rospy.is_shutdown()):
                #     rate.sleep()
                if(flag_transback):
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                    pose.pose.position.z = 20
                    local_pos_pub.publish(pose)
                    

        else:
            if(current_pose_gazebo.pose.position.x<200):
                if(flag_transition):
                    pose.pose.position.x = current_pose.pose.position.x + 20
                    pose.pose.position.y = 0
                    pose.pose.position.z = 20
                local_pos_pub.publish(pose)
            else:
                flag_transback = vtol_transtion(3)
                if(flag_transback):
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                    pose.pose.position.z = 20
                    local_pos_pub.publish(pose)

        rate.sleep()