#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
from time import sleep

import rospy
from rosserial_arduino import SerialClient
from serial import SerialException

if __name__ == "__main__":

    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param("~port", "/dev/ttyACM0")
    baud = int(rospy.get_param("~baud", "57600"))

    # Number of seconds of sync failure after which Arduino is auto-reset.
    # 0 = no timeout, auto-reset disabled
    auto_reset_timeout = int(rospy.get_param("~auto_reset_timeout", "0"))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param("~fix_pyserial_for_test", False)

    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2:
        port_name = sys.argv[1]

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(
                port_name,
                baud,
                fix_pyserial_for_test=fix_pyserial_for_test,
                auto_reset_timeout=auto_reset_timeout,
            )
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue