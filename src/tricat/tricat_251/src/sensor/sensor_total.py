#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
이 코드는 ROS 노드로, 세 가지 주요 센서 데이터를 수집하고 이를 통합하여 Sensor_total 메시지로 발행

구성 요소:
1. /obstacles 토픽: ObstacleList 타입의 메시지를 수신하여 장애물 정보를 저장
2. /position_ned 토픽: Point 타입의 메시지를 수신하여 NED 좌표계 상의 위치 정보를 저장(필요시 enu좌표계 사용할것)
3. /psi 토픽: Float64 타입의 메시지를 수신하여 방향 정보를 저장합니다.

기능:
- 위치 데이터는 이전 값과 비교하여 새로 수신한 데이터가 1 이상의 차이가 있는 경우(값은 실험을 통해 찾을 예정)
  - 이전 값과 새 값의 평균을 사용하여 보정합니다.-- 평균으로 하는게 맞는지 이전값을 사용하는게 맞는지는 각자 판단
  - 보정이 발생하면 터미널에 경고 메시지를 출력

- Sensor_total 메시지를 10Hz로 발행하며, 모든 필수 데이터가 수집되었는지 확인한 후 발행합니다.

주의사항:
- 모든 필수 데이터를 수신하지 못한 경우, 발행이 지연되며 터미널에 알림이 출력됩니다.
"""

import rospy
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Point
from tricat_msgs.msg import ObstacleList, Sensor_total as SensorTotalMsg

class SensorTotalNode:
    def __init__(self):
        # Initialize variables to store received data
        self.obstacles = None
        self.psi = None
        self.position_ned_x = None
        self.position_ned_y = None
        self.prev_position_ned_x = None
        self.prev_position_ned_y = None

        # Setup subscribers
        # rospy.Subscriber("/obstacles", ObstacleList, self.obstacles_callback, queue_size=10)
        rospy.Subscriber("/position_ned", Point, self.position_ned_callback, queue_size=10)
        rospy.Subscriber("/psi", Float64, self.psi_callback, queue_size=10)

        # Setup publisher
        self.sensor_total_pub = rospy.Publisher("/sensor_total", SensorTotalMsg, queue_size=10)

        # Set a timer to periodically publish the message
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_sensor_total)

    # def obstacles_callback(self, msg):
    #     self.obstacles = msg  # Directly assign the message

    def psi_callback(self, msg):
        self.psi = msg.data  # Directly assign the psi value

    def position_ned_callback(self, msg):
        # Check if previous positions exist for comparison
        if self.prev_position_ned_x is not None and self.prev_position_ned_y is not None:
            # Calculate differences
            x_diff = abs(msg.x - self.prev_position_ned_x)
            y_diff = abs(msg.y - self.prev_position_ned_y)

            # Check if differences exceed the threshold of 1
            if x_diff > 1 or y_diff > 1:
                # Calculate the average of the previous and current positions
                self.position_ned_x = (self.prev_position_ned_x + msg.x) / 2
                self.position_ned_y = (self.prev_position_ned_y + msg.y) / 2
                rospy.logwarn("GPS value out of range, replacing with average")
            else:
                self.position_ned_x = msg.x
                self.position_ned_y = msg.y
        else:
            # Directly assign the first set of position data
            self.position_ned_x = msg.x
            self.position_ned_y = msg.y

        # Update previous position for future comparisons
        self.prev_position_ned_x = self.position_ned_x
        self.prev_position_ned_y = self.position_ned_y

    def publish_sensor_total(self, event):
        # Ensure all required data is available
        missing_data = []
        # if self.obstacles is None:
        #     missing_data.append('obstacles')
        if self.psi is None:  # Check if we have received a psi value
            missing_data.append('psi')
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
            # sensor_total_msg.obstacles = self.obstacles  # Directly assign the ObstacleList message

            self.sensor_total_pub.publish(sensor_total_msg)
            rospy.loginfo("Sensor_total message published")
        else:
            rospy.loginfo(f"Data not complete for publishing Sensor_total message, missing data: {', '.join(missing_data)}")

def main():
    rospy.init_node('sensor_total_node')
    node = SensorTotalNode()
    rospy.spin()

if __name__ == "__main__":
    main()
