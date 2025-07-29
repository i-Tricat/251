#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import heapq
import numpy as np
import pymap3d as pm
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
            n, e, d = pm.geodetic2ned(waypoint[0], waypoint[1], waypoint[2], self.origin[0], self.origin[1], self.origin[2])

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


class GlobalPathPlanner:
    def __init__(self, map):
        self.cost_map = map
    
    @staticmethod
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    @staticmethod
    def get_neighbors(node, rows, cols):
        neighbors = []
        for d in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (node[0] + d[0], node[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                neighbors.append(neighbor)
        return neighbors

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star_search(self, start, goal):
        rows, cols = self.cost_map.shape
        open_list = []
        open_set = set()
        heapq.heappush(open_list, (0, start))
        open_set.add(start)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_list:
            current = heapq.heappop(open_list)[1]
            open_set.remove(current)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current, rows, cols):
                tentative_g_score = g_score[current] + self.cost_map[neighbor]
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)
        return []


if __name__ == "__main__":
    wp_server = WpServer()
    wp_server.wp_service_server()