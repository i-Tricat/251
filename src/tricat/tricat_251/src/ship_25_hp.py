#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import UInt16
from math import pi, degrees
import math
import numpy as np
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from ship.wp_manager import WpManager
from control.autopilot import heading_cal

# === 추가: IMU 구독(요레이트) ===
from sensor_msgs.msg import Imu

##### 시각화 #####
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

##### 터미널 색 #####
from termcolor import colored

D2R = pi / 180.0
R2D = 180.0 / pi

class SHIP:
    def __init__(self):
        # === Waypoint ===
        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_k = []
        self.d_goal = 0.0
        self.target_heading = 0.0  # [rad]
        if self.wp_manager.WP_data:
            # 내부 정책에 따라 초기화 루틴이 있을 때 사용
            try:
                self.wp_manager.initialize()
            except Exception:
                pass

        # === 상태 ===
        self.control_mode = "Avoidance"
        self.control_angle = 0.0     # [deg] 표시용
        self.psi_desire   = 0.0      # [deg] 표시용

        # === Pose ===
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0  # [rad]
        self.r_ned = 0.0    # [rad/s] (IMU에서 갱신)
        rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)

        # === IMU (요레이트) ===
        self.use_imu_yaw_rate = rospy.get_param("use_imu_yaw_rate", True)
        if self.use_imu_yaw_rate:
            rospy.Subscriber("/imu/data", Imu, self.imu_callback, queue_size=10)

        # === LiDAR / Obstacles ===
        rospy.Subscriber("/obstacles", Obstacles, self.obstacle_callback, queue_size=10)
        self.range            = rospy.get_param("range", 10.0)               # 회피 계산 반경
        self.display_range    = rospy.get_param("display_range", self.range) # 시각화 전용 반경
        self.yaw_range        = rospy.get_param("yaw_range", 60.0)           # [deg] FOV=헤딩±yaw/2
        self.margin           = rospy.get_param("margin", 0.5)               # [m] 장애물 팽창
        self.goal_range       = rospy.get_param("goal_range", 2.0)
        self.min_free_gap_deg = rospy.get_param("min_free_gap_deg", 5.0)     # [deg] 너무 좁은 자유구간 버림
        self.obstacles = []
        self.last_obstacles_stamp = rospy.Time(0)
        self.obstacle_timeout_sec = rospy.get_param("obstacle_timeout_sec", 0.8)

        # 시각화용 저장 (월드 라디안)
        self._free_intervals_world = []
        self._last_gap_center_rad = None
        self._merged_blocks_world = []   # <<< 추가: 차단구간(FOV내 병합) 저장

        # === Control ===
        self.control_msg  = Control()
        self.control_pub  = rospy.Publisher("/Control", Control, queue_size=1)
        self.thruster_s   = 1500
        self.thruster_p   = 1500

        # PD 및 구동 파라미터
        self.base_thrust      = rospy.get_param("base_thrust", 1500)
        self.thrust_range     = rospy.get_param("thrust_range", [1100, 1900])
        self.kp_thruster      = rospy.get_param("kp_thruster", 1.7)
        self.kd_thruster      = rospy.get_param("kd_thruster", 0.3)

        # 스핀/회피 보조 파라미터
        self.spin_threshold_deg = rospy.get_param("spin_threshold_deg", 10.0)
        self.spin_pwm_diff      = rospy.get_param("spin_pwm_diff", 150)

        # 자유구간 없음 Fallback 파라미터
        self.no_gap_count             = 0
        self.no_gap_trigger_count     = rospy.get_param("no_gap_trigger_count", 6)   # 6주기 연속(0.6s@10Hz)
        self.reverse_pwm              = rospy.get_param("reverse_pwm", 1400)
        self.reverse_duration_sec     = rospy.get_param("reverse_duration_sec", 0.7) # 후진 시간
        self.spin_escape_duration_sec = rospy.get_param("spin_escape_duration_sec", 1.2)
        self._escape_until = rospy.Time(0)
        self._escape_mode  = None  # "reverse" 또는 "spin"

        # 자유구간 선택 비용(폭-목표거리) 가중치
        self.goal_bias_weight = rospy.get_param("goal_bias_weight", 0.6)  # [rad] 가중 항

        # === 시각화 ===
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.arrival_range = rospy.get_param("goal_range", 10.0)

    # ================= 콜백 =================
    def pose_callback(self, msg):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)
            psi_deg = float(msg.psi.data)
            self.psi_ned = ((psi_deg * D2R + pi) % (2 * pi)) - pi  # [-pi,pi)
            rospy.loginfo_throttle(2, f"Pose: x={self.x_ned:.3f}, y={self.y_ned:.3f}, psi(deg)={psi_deg:.2f}")
            self.publish_tf(None)
        except Exception as e:
            rospy.logwarn(f"Pose 수신 오류: {e}")

    def imu_callback(self, msg: Imu):
        try:
            # z축 각속도(rad/s)
            self.r_ned = float(msg.angular_velocity.z)
        except Exception as e:
            rospy.logwarn_throttle(2, f"IMU 수신 오류: {e}")

    def obstacle_callback(self, msg):
        self.obstacles = msg.circles + msg.segments
        self.last_obstacles_stamp = rospy.Time.now()

    # ================= 각/구간 유틸 =================
    def _wrap_pi(self, ang):
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def _angle_interval_union(self, intervals):
        if not intervals:
            return []
        intervals.sort(key=lambda x: x[0])
        merged = [list(intervals[0])]
        for a, b in intervals[1:]:
            if a <= merged[-1][1] + 1e-9:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        return [(a, b) for a, b in merged]

    def _clip_to_fov(self, a, b, fov_min, fov_max):
        if b < fov_min or a > fov_max:
            return []
        return [(max(a, fov_min), min(b, fov_max))]

    def _normalize_intervals_to_fov(self, intervals, psi_center_rad, yaw_range_deg):
        yaw = math.radians(yaw_range_deg)
        fov_min = self._wrap_pi(psi_center_rad - yaw / 2.0)
        fov_max = self._wrap_pi(psi_center_rad + yaw / 2.0)

        def unwrap_ref(theta, ref):
            t = theta
            while t - ref <= -math.pi:
                t += 2 * math.pi
            while t - ref > math.pi:
                t -= 2 * math.pi
            return t

        fmin = fov_min
        fmax = unwrap_ref(fov_max, fmin)

        norm = []
        for a, b in intervals:
            a = unwrap_ref(a, fmin)
            b = unwrap_ref(b, fmin)
            if b < a:
                a, b = b, a
            norm += self._clip_to_fov(a, b, fmin, fmax)

        merged = self._angle_interval_union(norm)
        return merged, fmin, fmax

    def _transform_obstacles_to_world(self, psi_ned, x_ned, y_ned):
        """
        센서/선체 -> 월드 변환.
        (관례: -local_x, +local_y 가 전방/좌현)
        """
        static_OB_data = []
        c = math.cos(psi_ned)
        s = math.sin(psi_ned)

        for obstacle in self.obstacles:
            if hasattr(obstacle, 'first_point') and hasattr(obstacle, 'last_point'):
                bx = x_ned + (-obstacle.first_point.x) * c - (obstacle.first_point.y) * s
                by = y_ned + (-obstacle.first_point.x) * s + (obstacle.first_point.y) * c
                ex = x_ned + (-obstacle.last_point.x)  * c - (obstacle.last_point.y)  * s
                ey = y_ned + (-obstacle.last_point.x)  * s + (obstacle.last_point.y)  * c
                static_OB_data.append(('segment', bx, by, ex, ey))
            elif hasattr(obstacle, 'center') and hasattr(obstacle, 'radius'):
                cx = x_ned + (-obstacle.center.x) * c - (obstacle.center.y) * s
                cy = y_ned + (-obstacle.center.x) * s + (obstacle.center.y) * c
                static_OB_data.append(('circle', cx, cy, max(0.0, float(obstacle.radius))))
        return static_OB_data

    # ================= 면(각도구간) 기반 회피 =================
    def _blocked_intervals_from_obstacles(self):
        # 장애물 타임아웃: 오래된 데이터는 무시
        if (rospy.Time.now() - self.last_obstacles_stamp).to_sec() > self.obstacle_timeout_sec:
            return []

        blocks = []
        obs = self._transform_obstacles_to_world(self.psi_ned, self.x_ned, self.y_ned)

        for ob in obs:
            if ob[0] == 'circle':
                _, cx, cy, r = ob
                dx = cx - self.x_ned
                dy = cy - self.y_ned
                d = math.hypot(dx, dy)
                if d > self.range + r + self.margin:
                    continue

                R = r + self.margin
                if d <= 1e-6 or d <= R:
                    # 거의 접촉 → 전방 전부 차단
                    blocks.append((-math.pi, math.pi))
                    continue

                theta_c = math.atan2(dy, dx)
                phi = math.acos(max(-1.0, min(1.0, R / d)))
                a = self._wrap_pi(theta_c - phi)
                b = self._wrap_pi(theta_c + phi)
                if (b - a) % (2 * math.pi) < 0:
                    a, b = b, a
                blocks.append((a, b))

            elif ob[0] == 'segment':
                _, x1, y1, x2, y2 = ob
                # 양 끝점 각도
                dx1, dy1 = x1 - self.x_ned, y1 - self.y_ned
                dx2, dy2 = x2 - self.x_ned, y2 - self.y_ned
                d1 = math.hypot(dx1, dy1)
                d2 = math.hypot(dx2, dy2)
                if d1 > self.range and d2 > self.range:
                    continue

                th1 = math.atan2(dy1, dx1)
                th2 = math.atan2(dy2, dx2)
                a = self._wrap_pi(min(th1, th2))
                b = self._wrap_pi(max(th1, th2))
                if (b - a) % (2 * math.pi) < 0:
                    a, b = b, a

                # 거리 의존 각도 팽창: 가까울수록 더 크게
                nearest_d = max(1e-3, min(d1, d2))
                expand = math.atan2(self.margin, max(1.0, nearest_d))
                a = self._wrap_pi(a - expand)
                b = self._wrap_pi(b + expand)
                blocks.append((a, b))

        return blocks

    def plan_avoidance(self):
        """
        차단 합집합 → FOV 내 여집합 → 비용함수(폭-목표각차*가중) 최대 구간의 중앙각 선택.
        반환: psi_desire_deg
        """
        if len(self.WP_k) < 2:
            rospy.logwarn_throttle(2, "웨이포인트 부족(plan_avoidance)")
            # 센서 전방 기준 유지
            psi_center = self._wrap_pi(self.psi_ned + math.pi)
            return ((math.degrees(psi_center) + 180.0) % 360.0) - 180.0

        # 센서 변환 관례(-local_x 전방) 보정
        psi_center = self._wrap_pi(self.psi_ned + math.pi)

        self.target_heading = heading_cal(self.WP_k[1].x.data,
                                          self.WP_k[1].y.data,
                                          self.x_ned, self.y_ned)
        target_rad = self.target_heading

        blocks = self._blocked_intervals_from_obstacles()
        merged_blocks, fmin, fmax = self._normalize_intervals_to_fov(blocks, psi_center, self.yaw_range)

        # === 시각화 저장: 차단구간(FOV 내 병합 결과) ===
        self._merged_blocks_world = [(self._wrap_pi(a), self._wrap_pi(b)) for (a, b) in merged_blocks]

        # 자유구간 계산
        free_intervals = []
        cur = fmin
        for a, b in merged_blocks:
            if a > cur:
                free_intervals.append((cur, a))
            cur = max(cur, b)
        if cur < fmax:
            free_intervals.append((cur, fmax))

        # 너무 좁은 자유구간 제거
        min_len = math.radians(self.min_free_gap_deg)
        free_intervals = [(a, b) for (a, b) in free_intervals if (b - a) >= min_len]

        # 시각화용 저장(월드 라디안)
        self._free_intervals_world = [(self._wrap_pi(a), self._wrap_pi(b)) for (a, b) in free_intervals]

        # 최적 자유구간 중앙 선택: 폭 - goal_bias_weight * |center - target_on_fov|
        self._last_gap_center_rad = None
        if free_intervals:
            self.no_gap_count = 0
            best_center = None
            best_score = -1e9

            # FOV 기준으로 타깃을 언랩
            target_on_fov = ((target_rad - fmin + math.pi) % (2 * math.pi)) + fmin - math.pi

            for a, b in free_intervals:
                length = b - a                        # [rad]
                center = (a + b) / 2.0               # [rad]
                ang_err = abs(self._wrap_pi(center - target_on_fov))  # [rad]
                score = length - self.goal_bias_weight * ang_err      # 큰 값이 좋음

                if score > best_score:
                    best_score = score
                    best_center = center

            psi_desire_rad = self._wrap_pi(best_center)
            self._last_gap_center_rad = psi_desire_rad  # 시각화용(보라 화살표)
            psi_desire_deg = ((math.degrees(psi_desire_rad) + 180.0) % 360.0) - 180.0
            return psi_desire_deg

        # 자유구간 없음 → 현재 전방 유지 (추가: 상위 Fallback에서 처리)
        self.no_gap_count += 1
        fallback_rad = psi_center
        return ((math.degrees(fallback_rad) + 180.0) % 360.0) - 180.0

    # ================= 제어 =================
    def _apply_escape_fallback(self):
        """자유구간 없음이 지속될 때 실행되는 안전 탈출 동작."""
        now = rospy.Time.now()

        # 이미 탈출 모드면 끝날 때까지 유지
        if now < self._escape_until:
            if self._escape_mode == "reverse":
                self.thruster_p = self.reverse_pwm
                self.thruster_s = self.reverse_pwm
            elif self._escape_mode == "spin":
                # 제자리 회전(좌현 기준)
                self.thruster_p = 1500 + self.spin_pwm_diff
                self.thruster_s = 1500 - self.spin_pwm_diff
            return True

        # 새로 진입 판단
        if self.no_gap_count >= self.no_gap_trigger_count:
            # 1단계: 후진
            self._escape_mode  = "reverse"
            self._escape_until = now + rospy.Duration(self.reverse_duration_sec)
            return True

        return False

    def _heading_pd_control(self, psi_desire_deg):
        # Fallback(탈출모드) 우선 적용
        if self._apply_escape_fallback():
            return self.thruster_p, self.thruster_s

        # 전방(+pi 보정과 일치) 현재각도 사용
        cur_deg = ((degrees(self.psi_ned + math.pi) + 180.0) % 360.0) - 180.0
        control_angle = (psi_desire_deg - cur_deg + 180.0) % 360.0 - 180.0
        self.control_angle = control_angle

        # 후진 직후에는 제자리 회전으로 전환하여 시야 확보
        if self._escape_mode == "reverse" and rospy.Time.now() >= self._escape_until:
            self._escape_mode  = "spin"
            self._escape_until = rospy.Time.now() + rospy.Duration(self.spin_escape_duration_sec)

        # 스핀 임계각
        if abs(control_angle) >= self.spin_threshold_deg:
            if control_angle >= 0.0:
                self.thruster_p = 1500 + self.spin_pwm_diff
                self.thruster_s = 1500 - self.spin_pwm_diff
            else:
                self.thruster_p = 1500 - self.spin_pwm_diff
                self.thruster_s = 1500 + self.spin_pwm_diff
            return self.thruster_p, self.thruster_s

        # PD 제어 (yaw rate는 IMU에서 갱신)
        cp_thrust = self.kp_thruster * control_angle
        yaw_rate  = self.r_ned
        cd_thrust = self.kd_thruster * (-yaw_rate)
        thrust_diff = cp_thrust + cd_thrust

        base = self.base_thrust
        left  = base + thrust_diff
        right = base - thrust_diff

        self.thruster_p = max(min(left,  self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right, self.thrust_range[1]), self.thrust_range[0])
        # 탈출 모드 해제
        self._escape_mode = None
        self._escape_until = rospy.Time(0)
        return self.thruster_p, self.thruster_s

    # ================= 실행 =================
    def Avoidance_run(self):
        psi_desire_deg = self.plan_avoidance()
        self.psi_desire = psi_desire_deg
        self._heading_pd_control(psi_desire_deg)
        self.visualize(psi_desire_deg, self.x_ned, self.y_ned)

    # ================= 퍼블리시 =================
    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    # ================= 시각화 =================
    def publish_tf(self, event=None):
        tnow = rospy.Time.now()

        map_tf = TransformStamped()
        map_tf.header.stamp = tnow
        map_tf.header.frame_id = "world"
        map_tf.child_frame_id = "map"
        map_tf.transform.rotation.w = 1.0

        base_tf = TransformStamped()
        base_tf.header.stamp = tnow
        base_tf.header.frame_id = "map"
        base_tf.child_frame_id = "base_link"
        base_tf.transform.translation.x = self.x_ned
        base_tf.transform.translation.y = self.y_ned
        q = quaternion_from_euler(0, 0, self.psi_ned)
        base_tf.transform.rotation.x = q[0]
        base_tf.transform.rotation.y = q[1]
        base_tf.transform.rotation.z = q[2]
        base_tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform([map_tf, base_tf])

    def publish_scanner_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "scanner"
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def _add_sector_markers(self, marker_array, intervals, x, y, radius, ns="free_intervals", start_id=10000):
        nseg = 24
        for k, (a, b) in enumerate(intervals):
            if b <= a:
                continue

            # 면(부채꼴)
            fill = Marker()
            fill.header.frame_id = "map"
            fill.header.stamp = rospy.Time.now()
            fill.ns = ns
            fill.id = start_id + 2*k
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.pose.orientation.w = 1.0
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            fill.color.r, fill.color.g, fill.color.b, fill.color.a = 0.2, 1.0, 0.2, 0.25

            arc_pts = []
            for i in range(nseg + 1):
                th = a + (b - a) * (i / float(nseg))
                px = x + math.cos(th) * radius
                py = y + math.sin(th) * radius
                arc_pts.append(Point(px, py, 0.0))

            center = Point(x, y, 0.0)
            tri_pts = []
            for i in range(nseg):
                tri_pts.append(center)
                tri_pts.append(arc_pts[i])
                tri_pts.append(arc_pts[i + 1])
            fill.points = tri_pts
            marker_array.markers.append(fill)

            # 외곽선
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = rospy.Time.now()
            line.ns = ns + "_outline"
            line.id = start_id + 2*k + 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.05
            line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 1.0, 0.2, 0.9
            line.points = arc_pts
            marker_array.markers.append(line)

    # === 추가: 차단구간(빨강) 마커 ===
    def _add_block_markers(self, marker_array, intervals, x, y, radius, ns="blocked_intervals", start_id=20000):
        nseg = 24
        for k, (a, b) in enumerate(intervals):
            if b <= a:
                continue

            fill = Marker()
            fill.header.frame_id = "map"
            fill.header.stamp = rospy.Time.now()
            fill.ns = ns
            fill.id = start_id + 2*k
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.pose.orientation.w = 1.0
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            # 빨강 반투명
            fill.color.r, fill.color.g, fill.color.b, fill.color.a = 1.0, 0.2, 0.2, 0.15

            arc_pts = []
            for i in range(nseg + 1):
                th = a + (b - a) * (i / float(nseg))
                px = x + math.cos(th) * radius
                py = y + math.sin(th) * radius
                arc_pts.append(Point(px, py, 0.0))

            center = Point(x, y, 0.0)
            tri_pts = []
            for i in range(nseg):
                tri_pts.append(center)
                tri_pts.append(arc_pts[i])
                tri_pts.append(arc_pts[i + 1])
            fill.points = tri_pts
            marker_array.markers.append(fill)

    def visualize(self, psi_desire_deg, x_ned, y_ned):
        ma = MarkerArray()

        # 보트
        gps = Marker()
        gps.header.frame_id = "map"
        gps.header.stamp = rospy.Time.now()
        gps.ns = "gps_position"
        gps.id = 0
        gps.type = Marker.SPHERE
        gps.action = Marker.ADD
        gps.pose.position.x = x_ned
        gps.pose.position.y = y_ned
        gps.scale.x = gps.scale.y = gps.scale.z = 0.5
        gps.color.b = 1.0
        gps.color.a = 1.0
        ma.markers.append(gps)

        # 웨이포인트/범위
        for idx, wp in enumerate(self.wp_manager.WP_data):
            p = Marker()
            p.header.frame_id = "map"
            p.header.stamp = rospy.Time.now()
            p.ns = "waypoints"
            p.id = idx + 100
            p.type = Marker.SPHERE
            p.action = Marker.ADD
            p.pose.position.x = wp.x.data
            p.pose.position.y = wp.y.data
            p.scale.x = p.scale.y = p.scale.z = 0.3
            if self.WP_k and len(self.WP_k) >= 2 and wp.num.data == self.WP_k[1].num.data:
                p.color.r = p.color.g = 1.0
            else:
                p.color.r = 1.0
            p.color.a = 1.0
            ma.markers.append(p)

            ring = Marker()
            ring.header.frame_id = "map"
            ring.header.stamp = rospy.Time.now()
            ring.ns = "waypoint_ranges"
            ring.id = idx + 1000
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = wp.x.data
            ring.pose.position.y = wp.y.data
            ring.scale.x = ring.scale.y = wp.range.data * 2.0
            ring.scale.z = 0.01
            ring.color.r = 0.5
            ring.color.a = 0.3
            ma.markers.append(ring)

        # === 차단구간(빨강) 먼저 표시 ===
        if self._merged_blocks_world:
            self._add_block_markers(ma, self._merged_blocks_world, x_ned, y_ned, radius=self.display_range)

        # === 자유구간(녹색) 표시 ===
        if self._free_intervals_world:
            self._add_sector_markers(ma, self._free_intervals_world, x_ned, y_ned, radius=self.display_range)

        # 제어 목표각(빨강 화살표)
        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = rospy.Time.now()
        arrow.ns = "desired_heading"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.15
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        st = Point(x_ned, y_ned, 0.0)
        rad = math.radians(psi_desire_deg)
        ed = Point(x_ned + math.cos(rad) * self.display_range,
                   y_ned + math.sin(rad) * self.display_range, 0.0)
        arrow.points = [st, ed]
        arrow.color.r = 1.0
        arrow.color.a = 1.0
        ma.markers.append(arrow)

        # 자유구간 중앙각(보라 화살표)
        if self._last_gap_center_rad is not None:
            mid = Marker()
            mid.header.frame_id = "map"
            mid.header.stamp = rospy.Time.now()
            mid.ns = "gap_center"
            mid.id = 2
            mid.type = Marker.ARROW
            mid.action = Marker.ADD
            mid.scale.x = 0.12
            mid.scale.y = 0.04
            mid.scale.z = 0.04
            st2 = Point(x_ned, y_ned, 0.0)
            ed2 = Point(x_ned + math.cos(self._last_gap_center_rad) * self.display_range,
                        y_ned + math.sin(self._last_gap_center_rad) * self.display_range, 0.0)
            mid.points = [st2, ed2]
            mid.color.r, mid.color.g, mid.color.b, mid.color.a = 0.6, 0.0, 0.8, 1.0
            ma.markers.append(mid)

        self.marker_array_pub.publish(ma)

    # ================= 로그 =================
    def print_state(self):
        sep = "=" * 50
        print(colored(sep, "cyan"))
        print(colored("현재 보트 상태", "yellow", attrs=["bold"]))
        print(colored(sep, "cyan"))

        print(f"{colored('제어 모드', 'green')}: {self.control_mode}")
        if len(self.WP_k) >= 2:
            print(f"{colored('다음 WP', 'green')}: #{self.WP_k[1].num.data} "
                  f"({self.WP_k[1].x.data:.2f}, {self.WP_k[1].y.data:.2f})")
        print(f"{colored('위치', 'green')}: ({self.x_ned:.2f}, {self.y_ned:.2f})")
        print(f"{colored('헤딩', 'blue')}: {degrees(self.psi_ned):.2f}°")
        print(f"{colored('선택각[psi_desire]', 'blue')}: {self.psi_desire:.2f}°")
        print(f"{colored('제어각오차', 'yellow')}: {self.control_angle:.2f}°")
        print(f"{colored('추진 PWM', 'red')}: L={int(self.thruster_p)}, R={int(self.thruster_s)}")

        # === 추가된 정보 ===
        print(f"{colored('인식된 장애물 개수', 'magenta')}: {len(self.obstacles)}")
        print(f"{colored('생성된 안전구역 개수', 'magenta')}: {len(self._free_intervals_world)}")
        if self._last_gap_center_rad is not None:
            print(f"{colored('선택된 안전구역 중앙각도', 'magenta')}: "
                  f"{math.degrees(self._last_gap_center_rad):.2f}°")
        else:
            print(f"{colored('선택된 안전구역 중앙각도', 'magenta')}: 없음")

        print(colored(sep, "cyan"))


# ================= 메인 루프 =================
def main():
    rospy.init_node("ship", anonymous=True)
    rate = rospy.Rate(10)
    ship = SHIP()
    while not rospy.is_shutdown():
        ship.publish_tf(None)
        ship.publish_scanner_tf()

        # 웨이포인트 관리
        try:
            ship.d_goal = ship.wp_manager.cal_d_goal(ship.x_ned, ship.y_ned)
            ship.WP_k   = ship.wp_manager.manage(ship.x_ned, ship.y_ned)
        except Exception as e:
            rospy.logwarn_throttle(2, f"WP 관리 오류: {e}")

        ship.Avoidance_run()
        ship.control_publish()
        ship.print_state()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
