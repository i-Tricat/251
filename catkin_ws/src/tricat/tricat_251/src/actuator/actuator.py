#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16
from tricat_msgs.msg import Control

class ActuatorNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('actuator', anonymous=True)

        # Initialize actuator control variables
        self.servo_p = 0
        self.servo_s = 0
        self.thruster_p = 0
        self.thruster_s = 0

        # Setup the subscriber on the specified topic
        rospy.Subscriber("/Control", Control, self.control_callback, queue_size=1)

        # Setup publishers for each actuator control
        self.servo_p_pub = rospy.Publisher("Control/servo_p", UInt16, queue_size=1)
        self.servo_s_pub = rospy.Publisher("Control/servo_s", UInt16, queue_size=1)
        self.thruster_p_pub = rospy.Publisher("Control/thruster_p", UInt16, queue_size=1)
        self.thruster_s_pub = rospy.Publisher("Control/thruster_s", UInt16, queue_size=1)
    
    def control_callback(self, msg):
        # Assign message values to the actuator control variables
        self.servo_p = msg.servo_p.data
        self.servo_s = msg.servo_s.data
        self.thruster_p = msg.thruster_p.data
        self.thruster_s = msg.thruster_s.data

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Publish the control values
            self.servo_p_pub.publish(self.servo_p)
            self.servo_s_pub.publish(self.servo_s)
            self.thruster_p_pub.publish(self.thruster_p)
            self.thruster_s_pub.publish(self.thruster_s)

            rate.sleep()

if __name__ == '__main__':
    try:
        node = ActuatorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Actuator node terminated.")
