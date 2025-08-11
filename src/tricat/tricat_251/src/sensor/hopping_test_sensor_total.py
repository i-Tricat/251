#!/usr/bin/env python3
# -*- coding:utf-8 -*-

#기존 sensor_total.py에서 라이다 관련된거 다 지움 GPS랑 IMU 값을 센서 토탈 메세지 값 불러와서 어쩌고 함

import rospy
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Point
from tricat_msgs.msg import Sensor_total as SensorTotalMsg

class HoppingTestSensorTotalNode:
    def __init__(self):
        # Initialize variables to store received data
        self.psi = None
        self.position_ned_x = None
        self.position_ned_y = None
        self.prev_position_ned_x = None
        self.prev_position_ned_y = None

        # Setup subscribers
        rospy.Subscriber("/position_ned", Point, self.position_ned_callback, queue_size=10)
        rospy.Subscriber("/psi", Float64, self.psi_callback, queue_size=10)

        # Setup publisher
        self.sensor_total_pub = rospy.Publisher("/sensor_total", SensorTotalMsg, queue_size=10)

        # Set a timer to periodically publish the message
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_sensor_total)

    def psi_callback(self, msg):
        self.psi = msg.data  # Directly assign the psi value

    def position_ned_callback(self, msg):
        # # Check if previous positions exist for comparison
        # if self.position_ned_x is not None and self.position_ned_y is not None:
        #     # Calculate differences
        #     x_diff = abs(msg.x - self.prev_position_ned_x)
        #     y_diff = abs(msg.y - self.prev_position_ned_y)

        #     # # Check if differences exceed the threshold of 1
        #     # if x_diff > 1 or y_diff > 1:
        #     #     # Calculate the average of the previous and current positions
        #     #     self.position_ned_x = (self.prev_position_ned_x + msg.x) / 2
        #     #     self.position_ned_y = (self.prev_position_ned_y + msg.y) / 2
        #     #     rospy.logwarn("GPS value out of range, replacing with average")
        #     # else:
        #     self.position_ned_x = msg.x
        #     self.position_ned_y = msg.y
        # else:
        #     # Directly assign the first set of position data
        self.position_ned_x = msg.x
        self.position_ned_y = msg.y

        # # Update previous position for future comparisons
        # self.prev_position_ned_x = self.position_ned_x
        # self.prev_position_ned_y = self.position_ned_y

    def publish_sensor_total(self, event):
        # Ensure all required data is available
        missing_data = []
        # if self.obstacles is None:
        #     missing_data.append('obstacles')
        # if self.psi is None:  # Check if we have received a psi value
        #     missing_data.append('psi')
        if self.position_ned_x is None:
            missing_data.append('position_ned_x')
        if self.position_ned_y is None:
            missing_data.append('position_ned_y')

        if not missing_data:
            sensor_total_msg = SensorTotalMsg()

            # Fill the header
            sensor_total_msg.header = Header()
            sensor_total_msg.header.stamp = rospy.Time.now()

            # Fill the message with the received data
            sensor_total_msg.position_ned = Point(x=self.position_ned_x, y=self.position_ned_y)
            sensor_total_msg.psi = Float64(data=self.psi)
        



            self.sensor_total_pub.publish(sensor_total_msg)
            rospy.loginfo("Sensor_total message published")
        else:
            rospy.loginfo(f"Data not complete for publishing Sensor_total message, missing data: {', '.join(missing_data)}")

def main():
    rospy.init_node('HoppingTestSensorTotalNode')
    node = HoppingTestSensorTotalNode()
    rospy.spin()

if __name__ == "__main__":
    main()
