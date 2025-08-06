#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from tricat_msgs.srv import WaypointService, WaypointServiceResponse
from math import hypot

class WpManager():
    def __init__(self):
        self.WP_data = []
        self.WP_k = []
        self.num_k = 0
        self.d_goal = 0
        self.ch = False

    def wp_client(self):
        '''
        WP_data 받기
        '''
        rospy.wait_for_service('get_waypoints')
        try:
            get_waypoints = rospy.ServiceProxy('get_waypoints', WaypointService)
            response = get_waypoints()  # WaypointServiceResponse 객체를 반환
            self.WP_data = response.waypoint_list.WP_data  # 리스트로 할당
            self.num_k = 0

            rospy.loginfo("Received waypoint list:")
            for wp in response.waypoint_list.WP_data:
                rospy.loginfo(f"Waypoint {wp.num}: x={wp.x}, y={wp.y}")
            
            return self.WP_data

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def manage(self, x_ned, y_ned):
        '''
        WP_data에서 WP_k로 분리
        '''
        self.ch = False
        if not self.WP_data:
            rospy.logwarn("Waypoint data is empty. Please call wp_client first.")
            return
        
        if not self.WP_k:
            self.initialize()
        
        if self.num_k == len(self.WP_data) + 1:
            rospy.loginfo("All waypoints processed.")
            return

        self.d_goal = self.cal_d_goal(x_ned, y_ned)
        self.check()
        
        return self.WP_k

    def initialize(self):
        self.WP_k.append(self.WP_data[self.num_k])
        self.WP_k.append(self.WP_data[self.num_k + 1])
        self.print_update()

    def check(self):
        if self.d_goal <= self.WP_k[1].range.data:
            rospy.loginfo(f"Arrive at WP{self.WP_k[1].num.data}")
            rospy.sleep(3)
            self.num_k += 1
            self.WP_k.pop(0)

        if self.num_k >= len(self.WP_data):  # 마지막 웨이포인트까지 도달했으면
            self.num_k = 0  # 다시 처음부터
            rospy.loginfo("Looping back to first waypoint.")

        self.WP_k.append(self.WP_data[self.num_k])
        self.print_update()
        self.ch = True
    
    def cal_d_goal(self, x_ned, y_ned):
        return hypot(self.WP_k[1].x.data-x_ned, self.WP_k[1].y.data-y_ned)
    
    def print_update(self):
        rospy.loginfo("WP_k updated:")
        for i, waypoint in enumerate(self.WP_k[:2]):
            rospy.loginfo(f"WP_k Waypoint {waypoint.num}: x={waypoint.x}, y={waypoint.y}")
    
    
