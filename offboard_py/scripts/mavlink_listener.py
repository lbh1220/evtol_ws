#!/usr/bin/env python
import rospy
from pymavlink import mavutil
from std_msgs.msg import Float32

def mavlink_listener():
    # Initialize ROS node
    rospy.init_node('mavlink_listener_node', anonymous=True)
    # Create a ROS publisher
    pub = rospy.Publisher('/mavros/puller_motor_rpm', Float32, queue_size=10)

    # Connect to MAVLink (update IP and port as necessary)
    mav = mavutil.mavlink_connection('udp:127.0.0.1:14445')
    mav.wait_heartbeat()
    rospy.loginfo("MAVLink connection established!")

    # Loop to receive MAVLink messages
    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        msg = mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if msg:
            rpm = float(msg.servo5_raw) - 900.0
            servo_data = f"puller rotor Raw Output: {rpm}"
            # rospy.loginfo(servo_data)
            # Publish servo data as a ROS message
            pub.publish(rpm)
        rate.sleep()

if __name__ == '__main__':
    try:
        mavlink_listener()
    except rospy.ROSInterruptException:
        pass
