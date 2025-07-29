#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

class GnssDummyGenerator():
    def __init__(self):
        self.pub = rospy.Publisher('/ublox/fix', NavSatFix, queue_size=10)
        # self.gps = [37.4480443, 126.6534428, 29.624]
        # self.gps = [37.4480194, 126.6534689, 31.125]
        # self.gps = [37.4480025, 126.6534866, 33.741]
        # self.gps = [37.4480408, 126.6534907, 29.624]
        self.gps = [35.06965949, 128.57880311, 50.921]
        
        
    def gps_fix_publish(self):
        # Create a dummy NavSatFix message
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # Set dummy data
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        msg.latitude = self.gps[0] 
        msg.longitude = self.gps[1]
        msg.altitude = self.gps[2]
        
        msg.position_covariance = [0]*9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        # Publish the message
        self.pub.publish(msg)
    
def main():
    rospy.init_node("GnssDummyGenerator", anonymous=True)
    gnss_dummy_gen = GnssDummyGenerator()
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        gnss_dummy_gen.gps_fix_publish()
        rate.sleep()

if __name__ == "__main__":
    main()
