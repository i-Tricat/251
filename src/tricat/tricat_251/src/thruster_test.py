#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import UInt16

if __name__ == "__main__":
    rospy.init_node("thruster_fixed_test")

    # 퍼블리셔 생성
    self.control_publish  = rospy.Publisher("/Control", Control, queue_size=1)

    # === 여기서 원하는 값 입력 ===
    left_pwm  = 1600   # 왼쪽 쓰러스터: 전진
    right_pwm = 1400   # 오른쪽 쓰러스터: 후진

    rospy.loginfo(f"Publishing fixed PWM values: Left={left_pwm}, Right={right_pwm}")

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    rate = rospy.Rate(10)  # 10Hz 발행
    while not rospy.is_shutdown():
        control_publish()
        rate.sleep()
