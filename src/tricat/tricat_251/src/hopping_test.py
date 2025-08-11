#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16, Float64
from math import hypot, pi, degrees
import numpy as np
from geometry_msgs.msg import Point
from tricat_msgs.msg import Pose, Control
from ship.wp_manager import WpManager 
from control.autopilot import heading_cal
# from visual.visual_hopping import publish_tf, visualize
### 시각화 ###
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
D2R = pi / 180

class HoppingTestNode:
    def __init__(self):
        rospy.init_node('hopping_test_node')
        self.control_pub = rospy.Publisher('/Control', Control, queue_size=1)

        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_data = []
        self.WP_k = []
        self.num_k = 0
        self.d_goal = 0
        self.ch = False

        if self.wp_manager.WP_data:
            self.wp_manager.initialize()

        self.target_heading = 0.0
        self.target_angle = 0.0

        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.u_ned = 0.0
        self.v_ned = 0.0
        self.r_ned = 0.0
        self.U = 0.0

        self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=10)

        self.control_msg = Control()
        self.thruster_s = 0
        self.thruster_p = 0
        self.yaw_range = rospy.get_param("yaw_range", 70)
        self.base_thrust = rospy.get_param("base_thrust", 1500)
        self.thrust_range = rospy.get_param("thrust_range", [1350, 1650])
        self.kp_thruster = rospy.get_param("kp_thruster", 2.0)
        self.kd_thruster = rospy.get_param("kd_thruster", 0.3)
        # self.goal_range_2 = rospy.get_param("goal_range_slow")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        
    def pose_callback(self, msg):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)
            psi_deg = float(msg.psi.data)
            self.psi_ned = (((psi_deg) * D2R + pi) % (2 * pi)) - pi  # deg -> rad

            rospy.loginfo_throttle(2, f"Pose 수신: x={self.x_ned}, y={self.y_ned}, psi(deg)={psi_deg}, psi(rad)={self.psi_ned}")

            self.pose_received = True

        except Exception as e:
            rospy.logwarn(f"Pose 값 수신 오류: {e}")

    def hopping_run(self):
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)
        self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)
        self.target_angle = self.target_heading - self.psi_ned

        self.thruster_p, self.thruster_s = self.thrust_pid_controller2(self.psi_ned) 
        # self.thrust_pid_controller(self.psi_ned)
        self.publish_tf(None)
        self.visualize(self.x_ned, self.y_ned)
 
    
    def thrust_pid_controller2(self, psi_ned):
        psi_desire = self.target_heading
        control_angle = (psi_desire - psi_ned + pi) % (2 * pi) - pi
        control_angle_deg = degrees(control_angle)
        control_angle_deg = ((control_angle_deg) + 180) % 360 - 180
        self.psi_desire = psi_desire
        self.control_angle_deg = control_angle_deg
        # if vector_desired is None and psi_desire:
        #     self.thruster_p 
        #     self.thruster_s 
        if abs(control_angle_deg) > self.yaw_range:
            Re_diff = 150
            if 180 >control_angle_deg >= 0:
                self.thruster_p = 1500 - Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 + Re_diff
            elif -180 < control_angle_deg < 0:
                self.thruster_p = 1500 + Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 - Re_diff
        else:
            cp_thrust = self.kp_thruster * control_angle_deg
            yaw_rate = self.r_ned
            cd_thrust = self.kd_thruster * (-yaw_rate)

            thrust_diff = cp_thrust + cd_thrust

            base_thrust = self.base_thrust
            left_thrust = 3000 - (base_thrust + thrust_diff)
            right_thrust =3000 - (base_thrust - thrust_diff)

            # if self.d_goal <= self.goal_range_2:
            #     self.thruster_p -= 70
            #     self.thruster_s -= 70

            self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
            self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s
    
    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    def print_hopping(self):
        rospy.loginfo_throttle(1.0,"---------------------------------------------------------------------\n")
        rospy.loginfo_throttle(1.0,f"현재 웨이포인트: {self.WP_k[1].num.data}\n")
        rospy.loginfo_throttle(1.0,f"웨이포인트 위치: x={self.WP_k[1].x.data}, y={self.WP_k[1].y.data}\n")
        rospy.loginfo_throttle(1.0,f"목표까지 거리[d_goal: {self.d_goal:.3f} m\n")
        rospy.loginfo_throttle(1.0,f"현재 위치: x={self.x_ned}, y={self.y_ned}\n")
        rospy.loginfo_throttle(1.0,f"현재 방향: {degrees(self.psi_ned):.2f} deg\n")
        rospy.loginfo_throttle(1.0,f"목표 방향: {degrees(self.target_heading):.2f} deg\n")
        rospy.loginfo_throttle(1.0,f"추력 (왼쪽/오른쪽): {self.thruster_p:.0f} / {self.thruster_s:.0f}\n")

    def publish_tf(self, event=None):
        current_time = rospy.Time.now()

        # ✅ world → map 변환
        map_transform = TransformStamped()
        map_transform.header.stamp = current_time
        map_transform.header.frame_id = "world"
        map_transform.child_frame_id = "map"
        map_transform.transform.translation.x = 0.0
        map_transform.transform.translation.y = 0.0
        map_transform.transform.translation.z = 0.0
        map_transform.transform.rotation.x = 0.0
        map_transform.transform.rotation.y = 0.0
        map_transform.transform.rotation.z = 0.0
        map_transform.transform.rotation.w = 1.0

        # ✅ map → base_link 변환 (보트 위치)
        base_link_transform = TransformStamped()
        base_link_transform.header.stamp = current_time
        base_link_transform.header.frame_id = "map"
        base_link_transform.child_frame_id = "base_link"
        base_link_transform.transform.translation.x = self.x_ned
        base_link_transform.transform.translation.y = self.y_ned
        base_link_transform.transform.translation.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.psi_ned)
        base_link_transform.transform.rotation.x = quaternion[0]
        base_link_transform.transform.rotation.y = quaternion[1]
        base_link_transform.transform.rotation.z = quaternion[2]
        base_link_transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform([map_transform, base_link_transform])
        
    def visualize(self, x_ned, y_ned):
        # rospy.loginfo("🚀 visualize 호출됨")
        marker_array = MarkerArray()

        # ✅ 현재 위치 마커 (녹색 구)
        gps_marker = Marker()
        gps_marker.header.frame_id = "map"
        gps_marker.header.stamp = rospy.Time.now()
        gps_marker.ns = "gps_position"
        gps_marker.id = 0
        gps_marker.type = Marker.SPHERE
        gps_marker.action = Marker.ADD
        gps_marker.pose.position.x = x_ned
        gps_marker.pose.position.y = y_ned
        gps_marker.pose.position.z = 0
        gps_marker.scale.x = gps_marker.scale.y = gps_marker.scale.z = 0.5
        gps_marker.color.r = 0.0
        gps_marker.color.g = 0.0
        gps_marker.color.b = 1.0
        gps_marker.color.a = 1.0
        marker_array.markers.append(gps_marker)

        ### 웨이포인트 표시 및 도달범위 ###
        for idx, wp in enumerate(self.wp_manager.WP_data):
            # 🚩 웨이포인트 마커 (빨간색 점)
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = rospy.Time.now()
            wp_marker.ns = "waypoints"
            wp_marker.id = idx + 100  # ✅ ID 충돌 방지를 위해 고유 값 사용
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = wp.x.data
            wp_marker.pose.position.y = wp.y.data
            wp_marker.pose.position.z = 0
            wp_marker.scale.x = wp_marker.scale.y = wp_marker.scale.z = 0.3

            # 현재 목표 웨이포인트 강조 (노란색)
            if self.WP_k and wp.num.data == self.WP_k[1].num.data:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 1.0, 0.0, 1.0  # 노란색
            else:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 0.0, 0.0, 1.0  # 빨간색

            marker_array.markers.append(wp_marker)

            # 🔴 도달 범위 원 (반투명 빨간색)
            range_marker = Marker()
            range_marker.header.frame_id = "map"
            range_marker.header.stamp = rospy.Time.now()
            range_marker.ns = "waypoint_ranges"
            range_marker.id = idx + 1000  # ✅ 웨이포인트 ID와 겹치지 않도록 충분히 큰 수 사용
            range_marker.type = Marker.CYLINDER
            range_marker.action = Marker.ADD
            range_marker.pose.position.x = wp.x.data
            range_marker.pose.position.y = wp.y.data
            range_marker.pose.position.z = 0.0
            range_marker.scale.x = range_marker.scale.y = wp.range.data * 2  # 지름
            range_marker.scale.z = 0.01
            range_marker.color.r, range_marker.color.g, range_marker.color.b, range_marker.color.a = 0.5, 0.0, 0.0, 0.3
            marker_array.markers.append(range_marker)

        self.marker_array_pub.publish(marker_array)
# ✅ 파일 제일 아래에 위치 (클래스 밖)
if __name__ == '__main__':
    try:
        node = HoppingTestNode()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            node.hopping_run()
            node.control_publish()
            node.print_hopping()
    except rospy.ROSInterruptException:
        pass