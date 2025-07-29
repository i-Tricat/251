#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def broadcast_transforms():
    rospy.init_node('tf_broadcaster')

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # world_ned to base_link
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world_ned"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)  # Assuming no rotation between world_ned and base_link
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # base_link to left_hull
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "left_hull"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.4
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # base_link to right_hull
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "right_hull"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = -0.4
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # left_hull to servo1
        transform.header.frame_id = "left_hull"
        transform.child_frame_id = "servo1"
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.125
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # right_hull to servo2
        transform.header.frame_id = "right_hull"
        transform.child_frame_id = "servo2"
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.125
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # servo1 to thruster1
        transform.header.frame_id = "servo1"
        transform.child_frame_id = "thruster1"
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # servo2 to thruster2
        transform.header.frame_id = "servo2"
        transform.child_frame_id = "thruster2"
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # base_link to imu
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "imu"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # base_link to lidar
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "velodyne"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.3
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        # base_link to camera
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera"
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        br.sendTransform(transform)

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transforms()
    except rospy.ROSInterruptException:
        pass
