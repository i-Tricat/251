#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles, SegmentObstacle, CircleObstacle
from std_msgs.msg import Header
from geometry_msgs.msg import Point

class LidarDummyGenerator:
    def __init__(self):
        # Publisher for the obstacle topic
        self.pub = rospy.Publisher('/obstacles', Obstacles, queue_size=10)
        rospy.init_node('lidar_dummy_generator', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

    def generate_obstacles(self):
        obstacles = Obstacles()

        # Create a header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "scanner"
        obstacles.header = header

        # List of segment obstacles
        segment_data = [
            # [(3, 1), (-3, 1)],
            # Add the rest of the segments here in the same format
        ]

        # Add segments to the message
        for first, last in segment_data:
            segment = SegmentObstacle()
            segment.first_point = Point(first[0], first[1], 0.0)
            segment.last_point = Point(last[0], last[1], 0.0)
            obstacles.segments.append(segment)

        # List of circle obstacles
        circle_data = [
            (1.5,0,0.1),
            (1.5,1.5,0.1),
            (1.5,-1.5,0.1)
            # (2.5522128450248913, -2.789495001497416, 0.1598904045617896),
            # (2.892157899629629, -1.0908454451143081, 0.14762253532814051),
            # Add the rest of the circles here in the same format
        ]

        # Add circles to the message
        for center_x, center_y, radius in circle_data:
            circle = CircleObstacle()
            circle.center = Point(center_x, center_y, 0.0)
            circle.radius = radius
            obstacles.circles.append(circle)

        return obstacles

    def run(self):
        while not rospy.is_shutdown():
            obstacles = self.generate_obstacles()
            self.pub.publish(obstacles)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        generator = LidarDummyGenerator()
        generator.run()
    except rospy.ROSInterruptException:
        pass
