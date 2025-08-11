#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import heapq
import numpy as np
from pymap3d import geodetic2ned
from std_msgs.msg import Float64, String, UInt16, Bool
from sensor.gps import gnss_converter as gc
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
        for i, waypoint in enumerate(self.gnss_waypoint):
            # 이렇게 해도돼?
            n, e, d = geodetic2ned(waypoint[0], waypoint[1], waypoint[2], self.origin[0], self.origin[1], self.origin[2])

            waypoint_msg = WP()
            waypoint_msg.x = Float64(n)
            waypoint_msg.y = Float64(e)
            waypoint_msg.type = String("Must")
            waypoint_msg.num = UInt16(i)
            waypoint_msg.range = UInt16(self.goal_range)
            waypoint_msg.arrive = Bool(False)  # Assuming default value is False

            self.waypoint_list_msg.WP_data.append(waypoint_msg)
    
    def handle_waypoint_request(self, req):
        return WaypointServiceResponse(self.waypoint_list_msg)

    def wp_service_server(self):
        rospy.init_node('waypoint_service_server')
        rospy.Service('get_waypoints', WaypointService, self.handle_waypoint_request)
        rospy.loginfo("Waypoint service ready.")
        rospy.spin()


if __name__ == "__main__":
    wp_server = WpServer()
    wp_server.wp_service_server()