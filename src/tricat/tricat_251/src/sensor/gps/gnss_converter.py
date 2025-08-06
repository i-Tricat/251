#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64  # ← yaw(heading)용

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
        self.pub_name = '/position_ned' if self.type == 'ned' else '/position_enu'

        self.yaw_deg = 0.0  # ← 추가: yaw 값을 저장할 변수

        self.gnss_sub = rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_fix_callback, queue_size=1)
        self.yaw_sub = rospy.Subscriber("/psi", Float64, self.yaw_callback, queue_size=1)  # ← 추가: ψ 구독

        self.pub = rospy.Publisher(self.pub_name, Point, queue_size=10)
        self.point_pub = Point()
    
    def yaw_callback(self, msg):
        self.yaw_deg = msg.data  # ← heading 값을 저장

    def gps_fix_callback(self, msg):
        if self.type == 'ned':
            self.position[0], self.position[1], _ = ned_convert(self.origin, [msg.latitude, msg.longitude, msg.altitude])
        elif self.type == 'enu':
            self.position[0], self.position[1], _ = enu_convert(self.origin, [msg.latitude, msg.longitude, msg.altitude])

        # self.position[2] 는 고도 대신 yaw로 대체할 것임

    def publish(self):
        self.point_pub.x = self.position[0]
        self.point_pub.y = self.position[1]
        self.point_pub.z = self.yaw_deg  # ← 고도 대신 heading (deg) 퍼블리시
        self.pub.publish(self.point_pub)

def main():
    rospy.init_node("GnssConverter", anonymous=True)
    origin = rospy.get_param("origin")  # e.g., [37.123, 127.123, 0]
    gnss_converter = GnssConverter(origin, 'ned')
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        gnss_converter.publish()
        rate.sleep()

if __name__ == "__main__":
    main()
