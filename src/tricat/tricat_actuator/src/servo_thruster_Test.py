#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16

def thruster_publish_test():
    # Initialize the ROS node
    rospy.init_node('thruster_control', anonymous=True)
    
    # Setup the publisher on the specified topic
    thruster_p_pub = rospy.Publisher("/Control/thruster_p", UInt16, queue_size=1)
    thruster_s_pub = rospy.Publisher("/Control/thruster_s", UInt16, queue_size=1)
    
    # Set the publishing rate (e.g., 10 Hz)
    '''
    docking 
    thruster_p = 1460
    thruster_s = 1540

    boat speed check
    16 m
    -- 1650 -- 
    > test1: 13.23 s
    > test2: 13.15 s

    -- 1700 --
    > test1: 10.9 s
    > test2: 

    -- 1750 --
    > test1: 10 s
    > test2: 

    -- 1800 --
    > test1: 
    > test2: 

    -- 1900 --
    > test1: 
    > test2: 

    '''
    rate = rospy.Rate(10)  # 10 Hz
    thruster_p = 1515
    thruster_s = 1515
    rospy.loginfo("Thruster control node started, publishing to /Control/thruster_p")

    try:
        while not rospy.is_shutdown():
            # Publish the thruster value
            thruster_p_pub.publish(thruster_p)
            thruster_s_pub.publish(thruster_s)
            # Log the published value
            rospy.loginfo("Published: %d, %d", thruster_p, thruster_s)

            # Sleep to maintain the loop rate
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Thruster control node terminated.")



class ServoThrusterController:
    def __init__(self):
        rospy.init_node('servo_thruster_controller', anonymous=True)
        
        # 퍼블리셔 설정
        self.servo_p_pub = rospy.Publisher("/Control/servo_p", UInt16, queue_size=1)
        self.servo_s_pub = rospy.Publisher("/Control/servo_s", UInt16, queue_size=1)
        self.thruster_p_pub = rospy.Publisher("/Control/thruster_p", UInt16, queue_size=1)
        self.thruster_s_pub = rospy.Publisher("/Control/thruster_s", UInt16, queue_size=1)

        rospy.sleep(2)  # 잠시 대기하여 퍼블리셔가 설정될 시간을 줌

        self.rate = rospy.Rate(1)  # 1Hz로 퍼블리시
        self.run()

    def servo_callback(self, data):
        rospy.loginfo(f"Servo received: {data.data}")

    def thruster_callback(self, data):
        rospy.loginfo(f"Thruster received: {data.data}")

    def run(self):
        while not rospy.is_shutdown():
            self.stop_and_turn_left(120, 120, 1540, 1540) #왼쪽이 오른쪽,
            self.rate.sleep()

    def stop_and_turn_left(self, servo1, servo2, thruster1, thruster2):
        rospy.loginfo("Stopping and turning left")

        # 쓰러스터를 중지
        thrust1 = UInt16()
        thrust2 = UInt16()
        thrust1.data = thruster1
        thrust2.data = thruster2
        self.thruster_p_pub.publish(thrust1)
        self.thruster_s_pub.publish(thrust2)

        rospy.loginfo(f"Thrusters set to {thruster1} and {thruster2}")
        rospy.sleep(1)  # 1초 대기

        # 서보 모터를 설정된 각도로 회전
        servo_p = UInt16()
        servo_s = UInt16()
        servo_p.data = servo1
        servo_s.data = servo2
        self.servo_p_pub.publish(servo_p)
        self.servo_s_pub.publish(servo_s)

        

if __name__ == '__main__':
    try:
        controller = ServoThrusterController()
        #thruster_publish_test()
    except rospy.ROSInterruptException:
        pass

