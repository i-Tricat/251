#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16
from tricat_msgs.msg import Control

class StraightController:
    def __init__(self):
        rospy.init_node('straight_controller', anonymous=True)

        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)  # ← actuator가 구독하는 토픽

        rospy.sleep(1)  # 퍼블리셔 초기화 대기

        self.rate = rospy.Rate(10)  # 10 Hz
        self.thrust_value_p = 1600  # 직진용 PWM 값
        self.thrust_value_s = 1600
        self.run()

    def run(self):
        rospy.loginfo("Starting straight thrust via /Control message")
        while not rospy.is_shutdown():
            msg = Control()
            msg.thruster_p = UInt16(self.thrust_value_p)
            msg.thruster_s = UInt16(self.thrust_value_s)

            self.control_pub.publish(msg)
            rospy.loginfo(f"Published thrust (p/s): {self.thrust_value_p}, {self.thrust_value_s}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        StraightController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Straight control node terminated.")

