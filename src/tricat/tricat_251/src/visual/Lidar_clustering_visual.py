#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from math import sin, cos, radians, pi
from collections import deque # obstacle 부분에서 deque 자료구조 쓸 때 사용하는 거
import visual.show as sh 
from nav_msgs.msg import Path
from std_msgs.msg import Float64, UInt16
from geometry_msgs.msg import PointStamped, Point, Vector3, PoseStamped, PolygonStamped, Point32, Polygon
from visualization_msgs.msg import MarkerArray
from tricat_msgs.msg import ObstacleList
#import utils.gnss_converter as gc # gnss converter인데 이거 안쓰고 total,total_hyo에서 받아오고 싶은데
import numpy as np
# from urdf_parser_py.urdf import URDF

# ----- Lidar 시각화----------------------------------------------------------------------------------------------------------------------#
class obstacle_rviz:
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.threshold = 1000
        self.ids = deque(list(range(1,self.threshold)))
        self.obstacles = []
        self.psi = 0
        # sub
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.psi_sub = rospy.Subscriber("/psi", Float64 , self.psi_callback, queue_size=1)
        rospy.Subscriber('/obstacles',ObstacleList, self.obstacle_callback)
        rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        # pub
        self.rviz_pub = rospy.Publisher("/obstacles_rviz", MarkerArray, queue_size=10)
        
    def obstacle_callback(self,msg):
        self.obstacles = msg.obstacle

    def boat_position_callback(self, msg):
        self.boat_x = msg.x
        self.boat_y = msg.y

    def osbtacle_rviz(self):
        obstacle = []
        for ob in self.obstacles:
            begin_x = (self.boat_x + (-ob.begin.x) * cos(radians(self.psi)) - ob.begin.y * sin(radians(self.psi)))*-1
            begin_y = self.boat_y + (-ob.begin.x) * sin(radians(self.psi)) + ob.begin.y * cos(radians(self.psi))
            end_x = (self.boat_x + (-ob.end.x) * cos(radians(self.psi)) - ob.end.y * sin(radians(self.psi)))*-1
            end_y = self.boat_y + (-ob.end.x) * sin(radians(self.psi)) + ob.end.y * cos(radians(self.psi))
            obstacle.append([begin_x, begin_y])
            obstacle.append([end_x, end_y])
        ids = self.ids.pop()
        self.ids.append(ids)
        obstacle = sh.linelist_rviz(
            name="obstacle", id=ids, lines=obstacle, color_r=255, color_g=255, color_b=0, scale=0.1
        )
        return obstacle

    def publish_obstacle(self):
        obstacle = self.osbtacle_rviz()
        markers = sh.marker_array_rviz([obstacle])
        self.rviz_pub.publish(markers)
        
    def boat_position_callback(self,msg):
        self.boat_x = msg.x
        self.boat_y = msg.y
        
    def psi_callback(self, msg):
        self.psi = msg.data
#--------------------------------------------------------------------------------------------------------------------------------------------#


def main():
    rospy.init_node('visual_haechan')
    obstacle = obstacle_rviz()
    

    rate = rospy.Rate(10) # 10Hz
    try:
        while not rospy.is_shutdown():
            obstacle.publish_obstacle()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
