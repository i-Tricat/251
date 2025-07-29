#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import heapq
import numpy as np
from pymap3d import geodetic2enu, geodetic2ned
from std_msgs.msg import Float64, String, UInt16, Bool
from tricat_msgs.msg import WPList, WP
from tricat_msgs.srv import WaypointService, WaypointServiceResponse

class WpServer():
    def __init__(self):
        self.origin = rospy.get_param("origin")
        self.goal_range = rospy.get_param("goal_range")
        self.gnss_waypoint = rospy.get_param("waypoints")
        self.waypoint_list_msg = WPList()
        self.initialize()
    
    def initialize(self):
        rospy.loginfo(f"[DEBUG] waypoints: {self.gnss_waypoint}")
        rospy.loginfo(f"[DEBUG] origin: {self.origin}")
        for i, waypoint in enumerate(self.gnss_waypoint):
            # ENU 변환
            e_enu, n_enu, _ = geodetic2enu(
                waypoint[0], waypoint[1], waypoint[2],
                self.origin[0], self.origin[1], self.origin[2]
            )

            # NED 변환
            n_ned, e_ned, _ = geodetic2ned(
                waypoint[0], waypoint[1], waypoint[2],
                self.origin[0], self.origin[1], self.origin[2]
            )

            # 비교 로그 출력
            delta_e = abs(e_enu - e_ned)
            delta_n = abs(n_enu - n_ned)
            rospy.loginfo(
                f"[WP {i}] ENU(e={e_enu:.3f}, n={n_enu:.3f}) | NED(e={e_ned:.3f}, n={n_ned:.3f}) | Δe={delta_e:.3f}, Δn={delta_n:.3f}"
            )

            # ROS 메시지는 ENU 기준 사용
            waypoint_msg = WP()
            waypoint_msg.x = Float64(n_enu)  # north (ENU 기준)
            waypoint_msg.y = Float64(e_enu)  # east  (ENU 기준)
            waypoint_msg.type = String("Must")
            waypoint_msg.num = UInt16(i)
            waypoint_msg.range = UInt16(self.goal_range)
            waypoint_msg.arrive = Bool(False)

            self.waypoint_list_msg.WP_data.append(waypoint_msg)
    
    def handle_waypoint_request(self, req):
        return WaypointServiceResponse(self.waypoint_list_msg)

    def wp_service_server(self):
        rospy.init_node('waypoint_service_server')
        rospy.Service('get_waypoints', WaypointService, self.handle_waypoint_request)
        rospy.loginfo("Waypoint service ready.")

        # 유지 수단 추가
        rospy.Timer(rospy.Duration(1), lambda event: None)
        rospy.spin()

if __name__ == "__main__":
    wp_server = WpServer()
    wp_server.wp_service_server()