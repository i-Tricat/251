#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import UInt16, Float64
from math import hypot, pi, degrees, sqrt
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from tricat_msgs.msg import Pose, Control
from ship.wp_manager import WpManager
from control.autopilot import heading_cal

import tf2_ros
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

D2R = pi / 180

class HoppingTestNode:
    def __init__(self):
        rospy.init_node('hopping_test_node')
        self.control_pub = rospy.Publisher('/Control', Control, queue_size=1)

        # --- Waypoint Manager ---
        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_data = []
        self.WP_k = []
        self.num_k = 0
        self.d_goal = 0.0

        if self.wp_manager.WP_data:
            self.wp_manager.initialize()

        # --- States / Pose ---
        self.target_heading = 0.0
        self.target_angle = 0.0

        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0  # rad
        self.u_ned = 0.0
        self.v_ned = 0.0
        self.r_ned = 0.0
        self.U = 0.0

        self.pose_received = False

        self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=10)

        # --- Control message ---
        self.control_msg = Control()
        self.thruster_s = 1500
        self.thruster_p = 1500

        # --- Parameters ---
        # 제자리 회전 범위 -10도 ~ +10도
        self.spin_range_deg = rospy.get_param("spin_range_deg", 10.0)  # 제자리 회전 범위 (degrees)
        self.base_thrust = rospy.get_param("base_thrust", 1550)      # 전진 기본 추력
        self.thrust_range = rospy.get_param("thrust_range", [1350, 1650])

        # PID 게인 (HOP 상태에서만 사용)
        self.kp_thruster = rospy.get_param("kp_thruster", 2.0)
        self.kd_thruster = rospy.get_param("kd_thruster", 0.3)

        # TF/Marker
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # FSM 모드
        self.mode = "ROTATE"  # 초기엔 정렬부터

    # ----------------------- Callbacks -----------------------
    def pose_callback(self, msg: Pose):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)

            psi_deg = float(msg.psi.data)
            self.psi_ned = (((psi_deg) * D2R + pi) % (2 * pi)) - pi  # deg -> rad, [-pi, pi)

            # 선택: u, v, r 필드가 있으면 업데이트
            try:
                self.u_ned = float(msg.u.data)
                self.v_ned = float(msg.v.data)
            except Exception:
                pass
            try:
                self.r_ned = float(msg.r.data)
            except Exception:
                pass

            self.U = sqrt(self.u_ned**2 + self.v_ned**2)

            rospy.loginfo_throttle(2, f"Pose: x={self.x_ned:.2f}, y={self.y_ned:.2f}, psi(deg)={psi_deg:.2f}, r={self.r_ned:.3f}")
            self.pose_received = True
        except Exception as e:
            rospy.logwarn(f"Pose 값 수신 오류: {e}")

    # ----------------------- Core Loop -----------------------
    def hopping_run(self):
        # 최신 WP/거리 계산
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)

        # 목표 방위 갱신
        self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)  # rad
        self.target_angle = self.target_heading - self.psi_ned

        # 상태 전환 판단
        self.update_mode()

        # 상태별 추력 산출
        self.control_step()

        # 시각화
        self.publish_tf()
        self.visualize(self.x_ned, self.y_ned)

    def update_mode(self):
        # 각도 오차 계산 (deg, -180~180)
        control_angle = (self.target_heading - self.psi_ned + pi) % (2 * pi) - pi
        control_angle_deg = ((degrees(control_angle)) + 180) % 360 - 180
        self.control_angle_deg = control_angle_deg

        if abs(control_angle_deg) <= 15:
            # 제자리 회전 범위 내
            self.mode = "HOP"
        else:
            # 제자리 회전 범위 밖
            self.mode = "ROTATE"

    def control_step(self):
        if self.mode == "ROTATE":
            self.control_rotate()
        else:
            self.control_hop()

    # ----------------------- Controllers -----------------------
    def control_rotate(self):
        """
        제자리 회전: 좌/우 역추진만으로 회전. 각도 제한 없음.
        """
        # 각도 부호에 따라 좌우 반대로
        if self.control_angle_deg >= 0:
            self.thruster_p = 1500 + 100  # 역추진 (우측)
            self.thruster_s = 1500 - 100  # 역추진 (좌측)
        else:
            self.thruster_p = 1500 - 100  # 역추진 (좌측)
            self.thruster_s = 1500 + 100  # 역추진 (우측)

        # 안전 클램프
        self.thruster_p = int(max(min(self.thruster_p, self.thrust_range[1]), self.thrust_range[0]))
        self.thruster_s = int(max(min(self.thruster_s, self.thrust_range[1]), self.thrust_range[0]))

    def control_hop(self):
        """
        전진 추종(PD 차등): **각도 제한 완전 해제**.
        yaw_range 기반 분기 **완전 제거**, 오차만큼 좌우 차등을 주고 base_thrust로 밀어줌.
        """
        # PD (yaw rate가 제대로 들어오면 D 사용)
        cp = self.kp_thruster * self.control_angle_deg
        cd = self.kd_thruster * (-self.r_ned)

        thrust_diff = cp + cd

        left_thrust  = self.base_thrust + thrust_diff
        right_thrust = self.base_thrust - thrust_diff

        # 안전 클램프
        self.thruster_p = int(max(min(left_thrust,  self.thrust_range[1]), self.thrust_range[0]))
        self.thruster_s = int(max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0]))

    # ----------------------- I/O -----------------------
    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    def print_hopping(self):
        try:
            print("---------------------------------------------------------------------")
            print(f"모드: {self.mode}")
            print(f"현재 웨이포인트: {self.WP_k[1].num.data}")
            print(f"웨이포인트 위치: x={self.WP_k[1].x.data:.2f}, y={self.WP_k[1].y.data:.2f}")
            print(f"목표까지 거리 d_goal: {self.d_goal:.3f} m")
            print(f"현재 위치: x={self.x_ned:.2f}, y={self.y_ned:.2f}")
            print(f"현재 방향: {degrees(self.psi_ned):.2f} deg")
            print(f"목표 방향: {degrees(self.target_heading):.2f} deg")
            print(f"각도 오차: {self.control_angle_deg:.2f} deg")
            print(f"추력 (왼/오): {self.thruster_p} / {self.thruster_s}")
        except Exception:
            pass

    # ----------------------- Visualization -----------------------
    def publish_tf(self, event=None):
        current_time = rospy.Time.now()

        # world -> map
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

        # map -> base_link
        base_link_transform = TransformStamped()
        base_link_transform.header.stamp = current_time
        base_link_transform.header.frame_id = "map"
        base_link_transform.child_frame_id = "base_link"
        base_link_transform.transform.translation.x = self.x_ned
        base_link_transform.transform.translation.y = self.y_ned
        base_link_transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.psi_ned)
        base_link_transform.transform.rotation.x = q[0]
        base_link_transform.transform.rotation.y = q[1]
        base_link_transform.transform.rotation.z = q[2]
        base_link_transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform([map_transform, base_link_transform])

    def visualize(self, x_ned, y_ned):
        marker_array = MarkerArray()

        # 현재 위치 (파란 구)
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

        # 웨이포인트/범위
        for idx, wp in enumerate(self.wp_manager.WP_data):
            # WP 점
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = rospy.Time.now()
            wp_marker.ns = "waypoints"
            wp_marker.id = idx + 100
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = wp.x.data
            wp_marker.pose.position.y = wp.y.data
            wp_marker.pose.position.z = 0
            wp_marker.scale.x = wp_marker.scale.y = wp_marker.scale.z = 0.3

            if self.WP_k and wp.num.data == self.WP_k[1].num.data:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(wp_marker)

            # 도달 범위
            range_marker = Marker()
            range_marker.header.frame_id = "map"
            range_marker.header.stamp = rospy.Time.now()
            range_marker.ns = "waypoint_ranges"
            range_marker.id = idx + 1000
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

# ----------------------- Main -----------------------
if __name__ == '__main__':
    try:
        node = HoppingTestNode()
        node.WP_k = node.wp_manager.manage(node.x_ned, node.y_ned)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if node.pose_received:
                node.hopping_run()
                node.control_publish()
                node.print_hopping()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
