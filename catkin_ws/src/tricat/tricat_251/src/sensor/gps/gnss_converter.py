#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

def enu_convert(origin, gnss):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
        return e, n, u

def ned_convert(origin, gnss):
        n, e, d = pm.geodetic2ned(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
        return n, e, d

class GnssConverter():
    def __init__(self, origin, conversion_type='ned'):
        self.origin = origin
        self.position = [0, 0, 0]
        self.type = conversion_type
        self.pub_name = ''
        if self.type == 'ned':
            self.pub_name = '/position_ned'
        elif self.type == 'enu':
            self.pub_name = '/position_enu'

        self.gnss_sub = rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_fix_callback, queue_size=1)
        self.point_pub = Point()
        self.pub = rospy.Publisher(self.pub_name, Point, queue_size=10)
    
    def gps_fix_callback(self, msg):
        if self.type == 'ned':
            self.position[0], self.position[1], self.position[2] = ned_convert(self.origin, [msg.latitude, msg.longitude, msg.altitude])
        elif self.type == 'enu':
            self.position[0], self.position[1], self.position[2] = enu_convert(self.origin, [msg.latitude, msg.longitude, msg.altitude])

    def publish(self):
        self.point_pub.x = self.position[0]
        self.point_pub.y = self.position[1]
        self.point_pub.z = self.position[2]
        self.pub.publish(self.point_pub)


def main():
    rospy.init_node("GnssConverter", anonymous=True)
    origin = rospy.get_param("origin") # Reference point
    gnss_converter = GnssConverter(origin, 'ned')
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        gnss_converter.publish()
        rate.sleep()

if __name__ == "__main__":
    main()