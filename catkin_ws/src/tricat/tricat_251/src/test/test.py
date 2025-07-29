#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16
from tricat_msgs.msg import Pose, WPList, Control
from control.LOS import Alos

def ALOS_test():
    waypoints = {'x': [0, 10, 20], 'y': [0, 10, 20], 'k': [0, 1, 2], 'r': [1, 1, 1]}
    delta = 5
    gamma = 0.001
    h = 0.1

    alos = Alos(waypoints, delta, gamma, h)

    # Test initial waypoint
    x_ned, y_ned = 0, 0
    x_e, y_e, psi_d = alos.compute(x_ned, y_ned)
    print(f"Initial position: x_e = {x_e:.2f}, y_e = {y_e:.2f}, psi_d = {psi_d:.2f}")

    # Move to next waypoint
    x_ned, y_ned = 10, 10
    x_e, y_e, psi_d = alos.compute(x_ned, y_ned)
    print(f"Next waypoint: x_e = {x_e:.2f}, y_e = {y_e:.2f}, psi_d = {psi_d:.2f}")

    # Check if we switch to the next waypoint
    alos.check(x_e)
    print(f"Waypoint index after check: {alos.k}")

control_msg = Control()
servo_p = 98
servo_s = 98
thruster_p = 1600
thruster_s = 1600

control_pub = rospy.Publisher("/Control", Control, queue_size=1)

def control_publish():
        control_msg.servo_p = UInt16(servo_p)
        control_msg.servo_s = UInt16(servo_s)
        control_msg.thruster_p = UInt16(thruster_p)
        control_msg.thruster_s = UInt16(thruster_s)
        control_pub.publish(control_msg)

if __name__ == '__main__':
    # ALOS_test()
    rospy.init_node('test')
    while not rospy.is_shutdown():
        control_publish()
