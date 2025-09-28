#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import math
import numpy as np
from std_msgs.msg import UInt16
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from ship.wp_manager import WpManager
import tf2_ros

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

# --------- Utils ---------
def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

# ANSI color helpers
COL = {
    "reset":  "\033[0m",
    "bold":   "\033[1m",
    "red":    "\033[31m",
    "green":  "\033[32m",
    "yellow": "\033[33m",
    "blue":   "\033[34m",
    "mag":    "\033[35m",
    "cyan":   "\033[36m",
    "gray":   "\033[90m",
}
def color(txt, c="reset"):
    return f"{COL.get(c,'')}{txt}{COL['reset']}"

# --------- States ---------
STATE_NORMAL = "NORMAL"
STATE_BLOCK_REVERSE = "BLOCK_REVERSE"

class SHIP:
    def __init__(self):
        # === Waypoint ===
        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_k = []
        self.d_goal = 0.0
        if getattr(self.wp_manager, "WP_data", None):
            try:
                self.wp_manager.initialize()
            except Exception:
                pass

        # === Pose ===
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.r_ned = 0.0
        self.pose_received = False
        rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)

        # === LiDAR ===
        rospy.Subscriber("/obstacles", Obstacles, self.obstacles_callback, queue_size=10)
        self.obstacles = []

        # 좌표계/TF
        self.obstacles_frame = rospy.get_param("~obstacles_frame", "scanner")  # "scanner" | "world"
        self.map_frame  = rospy.get_param("~map_frame",  "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # LiDAR 파라미터 (센싱/표시 한계)
        self.lidar_min_exclusion  = rospy.get_param("~lidar_min_exclusion", 0.02)  # 너무 가까운 값 무시 [m]
        self.lidar_max_range      = rospy.get_param("~lidar_max_range", 4.5)       # 감지 최대 [m]
        self.lidar_yaw_range      = rospy.get_param("~lidar_yaw_range", 150.0)     # 총 FOV [deg]
        self.lidar_margin         = rospy.get_param("~lidar_margin", 0.30)         # 팽창 여유 [m]
        self.lidar_span_angle     = rospy.get_param("~lidar_span_angle", 20)       # 위험각 여유 [deg]

        # 제어/회피 전용 파라미터 (미설정 시 센싱 값과 동기화)
        self.control_yaw_range = rospy.get_param("~control_yaw_range", None)       # None이면 lidar_yaw_range 사용
        _avoid_range_param = rospy.get_param("~avoid_range", None)                 # None이면 lidar_max_range 사용
        self.avoid_range_m = float(_avoid_range_param) if _avoid_range_param is not None else float(self.lidar_max_range)

        # === Control ===
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.control_msg = Control()
        self.base_thrust  = rospy.get_param("~base_thrust", 1600)
        self.thrust_range = rospy.get_param("~thrust_range", [1300, 1700])
        self.kp_thruster  = rospy.get_param("~kp_thruster", 3.5)
        self.kd_thruster  = rospy.get_param("~kd_thruster", 0.3)
        self.thruster_p = self.base_thrust
        self.thruster_s = self.base_thrust
        self.prev_optimal = 0.0  # [deg], ship-body frame 기준

        # === 상태머신 파라미터 ===
        self.block_reverse_pwm = rospy.get_param("~block_reverse_pwm", 1350)  # 후진 세기
        self.block_reverse_duration = rospy.get_param("~block_reverse_duration", 1.0)  # [sec]
        self.block_reverse_log_throttle = rospy.get_param("~block_reverse_log_throttle", 0.5)  # 로그 간격

        # 상태 및 타이머
        self.sm_state = STATE_NORMAL
        self.block_until = rospy.Time(0)

        # Waypoint 도달 판정
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.5)

        # === RViz ===
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)

        # 시작 시 파라미터 요약 로그
        cy = (self.control_yaw_range if self.control_yaw_range is not None else self.lidar_yaw_range)
        rospy.loginfo(
            color(
                f"[PARAM] sense_yaw=±{self.lidar_yaw_range/2:.1f}°, control_yaw=±{cy/2:.1f}°, "
                f"sense_range={self.lidar_max_range:.2f} m, avoid_range={self.avoid_range_m:.2f} m",
                "cyan"
            )
        )

    # -------------------- Pose / TF --------------------
    def pose_callback(self, msg):
        self.x_ned = float(msg.x.data)
        self.y_ned = float(msg.y.data)
        psi_deg = float(msg.psi.data)
        self.psi_ned = wrap_pi(psi_deg * D2R)
        self.r_ned = float(msg.r.data) if hasattr(msg, "r") else 0.0
        self.pose_received = True

        # map -> base_link TF
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x_ned
        t.transform.translation.y = self.y_ned
        t.transform.translation.z = 0.0
        cy = math.cos(self.psi_ned / 2.0); sy = math.sin(self.psi_ned / 2.0)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

    # -------------------- Waypoints --------------------
    def update_waypoints(self):
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)

    def current_waypoint(self):
        if len(self.WP_k) >= 2:
            return np.array([self.WP_k[1].x.data, self.WP_k[1].y.data])
        return None

    # -------------------- LiDAR --------------------
    def obstacles_callback(self, msg):
        """센싱/표시 단계 필터: FOV, 센싱거리(lidar_max_range), 최소제외"""
        if not msg.circles:
            self.obstacles = []
            return

        fov_half = math.radians(self.lidar_yaw_range) / 2.0
        kept = []
        for c in msg.circles:
            if self.obstacles_frame == "scanner":
                # 센서 좌표계를 body(+x 전방, +y 좌현) 기준으로 맞추기 위한 보정
                bx, by = -c.center.x, -c.center.y
                d = math.hypot(bx, by)
                bearing = math.atan2(by, bx)
            else:
                dx, dy = c.center.x - self.x_ned, c.center.y - self.y_ned
                d = math.hypot(dx, dy)
                bearing = wrap_pi(math.atan2(dy, dx) - self.psi_ned)
                bx, by = dx, dy  # body 근사

            # 1차 필터: 거리 & FOV
            if d < self.lidar_min_exclusion or d > self.lidar_max_range:
                continue
            if abs(bearing) > fov_half:
                continue

            # 여유 반경 적용
            inflated_r = float(getattr(c, "radius", 0.0)) + self.lidar_margin
            c.radius = inflated_r
            # body frame 좌표 보존
            c.center.x, c.center.y = bx, by
            kept.append(c)

        self.obstacles = kept

    # -------------------- Safe/Danger 계산 --------------------
    def ob_calculate(self):
        """장애물 -> Danger 각도 집합, Safe 각도 집합 계산 (body frame, deg)"""
        # 제어용 각 범위(±half)
        ctrl_yaw = self.control_yaw_range if self.control_yaw_range is not None else self.lidar_yaw_range
        half = int(round(ctrl_yaw / 2.0))
        half = max(1, min(half, 179))  # 안전상 한계
        angle_range = (-half, half)

        danger = set()
        nearest = float('inf')

        for obs in self.obstacles:
            x, y, r = float(obs.center.x), float(obs.center.y), float(getattr(obs, "radius", 0.0))
            d = math.hypot(x, y)
            if d > self.avoid_range_m:
                continue  # 회피판단은 avoid_range 이내만 고려

            # 전방=0°, 좌 +, 우 - (x축 전방 기준)
            theta = int(round(math.degrees(math.atan2(y, x))))
            theta = (theta + 180) % 360 - 180  # [-180,180]

            # 각 반폭: asin(r/d) + 여유각
            denom = max(d, 1e-6)
            ratio = max(-1.0, min(1.0, r / denom))
            phi = int(round(math.degrees(math.asin(abs(ratio)))))
            span = phi + int(self.lidar_span_angle)

            begin_ang = max(angle_range[0], theta - span)
            end_ang   = min(angle_range[1], theta + span)
            if begin_ang <= end_ang:
                danger.update(range(begin_ang, end_ang + 1))
                nearest = min(nearest, d - r)

        safe = sorted(set(range(angle_range[0], angle_range[1] + 1)) - danger)
        nearest_val = None if nearest == float('inf') else max(0.0, nearest)
        return safe, sorted(danger), nearest_val

    def group_safe_angles(self, safe):
        if not safe:
            return []
        groups, cur = [], [safe[0]]
        for a in safe[1:]:
            if a == cur[-1] + 1:
                cur.append(a)
            else:
                groups.append(cur)
                cur = [a]
        groups.append(cur)
        return groups

    def calculate_group_centers(self, grouped):
        return [g[len(g)//2] for g in grouped]

    def optimal_cal(self, safe_angles, error_angle_deg):
        """에러각과 가장 가까운 안전 각도 선택 (deg, body frame). 안전각이 없으면 None."""
        if not safe_angles:
            return None
        return min(safe_angles, key=lambda c: abs(c - error_angle_deg))

    # -------------------- 상태머신 보조 --------------------
    def enter_block_reverse(self, reason: str = ""):
        """BLOCK_REVERSE 상태로 전이 + 타이머 설정"""
        self.sm_state = STATE_BLOCK_REVERSE
        self.block_until = rospy.Time.now() + rospy.Duration(self.block_reverse_duration)
        self.prev_optimal = 0.0  # D항 폭주 방지
        if reason:
            rospy.loginfo(color(f"[STATE] → BLOCK_REVERSE ({reason})", "mag"))
        else:
            rospy.loginfo(color("[STATE] → BLOCK_REVERSE", "mag"))

    def is_block_time_left(self) -> bool:
        return rospy.Time.now() < self.block_until

    # -------------------- Thruster 제어 --------------------
    def calculate_thruster(self, error_angle_deg, distance_to_goal):
        # 상태 체크: BLOCK_REVERSE이면 타이머 동안 무조건 후진 유지
        if self.sm_state == STATE_BLOCK_REVERSE:
            if self.is_block_time_left():
                self.thruster_p = self.block_reverse_pwm
                self.thruster_s = self.block_reverse_pwm
                rospy.loginfo_throttle(
                    self.block_reverse_log_throttle,
                    f"\n{color('[BLOCK-REV]', 'bold')} "
                    f"{color('remain', 'cyan')}={(self.block_until - rospy.Time.now()).to_sec():4.1f}s  "
                    f"{color('action', 'mag')}=REVERSE({self.block_reverse_pwm},{self.block_reverse_pwm})"
                )
                return
            else:
                self.sm_state = STATE_NORMAL
                rospy.loginfo(color("[STATE] BLOCK_REVERSE → NORMAL (timer done)", "green"))

        # NORMAL 상태: 안전각 기반 PD 제어
        safe, danger, nearest = self.ob_calculate()
        optimal_deg = self.optimal_cal(safe, error_angle_deg)

        # 안전각 없음 → BLOCK_REVERSE 진입
        if (not safe) or (optimal_deg is None):
            self.enter_block_reverse(reason="no safe angles")
            self.thruster_p = self.block_reverse_pwm
            self.thruster_s = self.block_reverse_pwm
            rospy.loginfo_throttle(
                self.block_reverse_log_throttle,
                f"\n{color('[BLOCKED]', 'bold')} "
                f"{color('dist', 'cyan')}={distance_to_goal:5.2f} m  "
                f"{color('err', 'yellow')}={error_angle_deg:6.1f}°  "
                f"{color('near', 'cyan')}={(nearest if nearest is not None else -1):5.2f} m  "
                f"{color('action', 'mag')}=REVERSE({self.block_reverse_pwm},{self.block_reverse_pwm})  "
                f"{color('safe#', 'gray')}={len(safe):3d}   "
                f"{color('dang#', 'mag')}={len(danger):3d}"
            )
            return

        # --- 정상 회피/추종 PD 제어 ---
        derivative = optimal_deg - self.prev_optimal
        pd_out = self.kp_thruster * optimal_deg + self.kd_thruster * derivative
        self.prev_optimal = optimal_deg

        base = self.base_thrust
        left  = np.clip(base + pd_out, *self.thrust_range)
        right = np.clip(base - pd_out, *self.thrust_range)

        self.thruster_p = int(left)
        self.thruster_s = int(right)

        log = (
            f"\n{color('[SAFE-NAV]', 'bold')} "
            f"{color('dist', 'cyan')}={distance_to_goal:5.2f} m  "
            f"{color('err', 'yellow')}={error_angle_deg:6.1f}°  "
            f"{color('opt', 'green')}={optimal_deg:6.1f}°  "
            f"{color('near', 'cyan')}={(nearest if nearest is not None else -1):5.2f} m\n"
            f"{color('P', 'blue')}={self.thruster_p:4d}   "
            f"{color('S', 'blue')}={self.thruster_s:4d}   "
            f"{color('safe#', 'gray')}={len(safe):3d}   "
            f"{color('dang#', 'mag')}={len(danger):3d}"
        )
        rospy.loginfo_throttle(0.5, log)

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    # -------------------- 목표 도달 처리 --------------------
    def reverse_and_rotate_in_place(self):
        rospy.loginfo(color("→ 목표 도달: 정지/후진/제자리 회전 수행", "mag"))

        # 1) 잠시 정지
        self.thruster_p = 1470
        self.thruster_s = 1470
        self.control_publish()
        rospy.sleep(1.0)

        # 2) 후진
        self.thruster_p = 1350
        self.thruster_s = 1350
        self.control_publish()
        rospy.sleep(1.0)

        # 3) 제자리 좌회전
        initial_heading_deg = math.degrees(self.psi_ned)
        while not rospy.is_shutdown():
            self.thruster_p = 1400  # 좌 후진
            self.thruster_s = 1580  # 우 전진
            self.control_publish()
            rospy.sleep(0.1)

            now_deg = math.degrees(self.psi_ned)
            diff = (now_deg - initial_heading_deg) % 360.0
            if diff > 180.0:
                diff = 360.0 - diff
            if abs(diff) > 10.0:
                break

        # 4) 다시 정지
        self.thruster_p = 1470
        self.thruster_s = 1470
        self.control_publish()
        rospy.sleep(1.0)

    # -------------------- Main Loop --------------------
    def detection_run(self):
        self.update_waypoints()
        wp_goal = self.current_waypoint()
        if wp_goal is None:
            rospy.logwarn_throttle(1.0, color("Waypoint 없음 → 정지 유지", "red"))
            self.thruster_p = self.base_thrust
            self.thruster_s = self.base_thrust
            self.control_publish()
            self.visualize_state(None, None)
            return

        # 목표각/거리 (NED: atan2(E, N))
        dx, dy = wp_goal[0] - self.x_ned, wp_goal[1] - self.y_ned
        dN = dx
        dE = dy
        goal_angle_deg = math.degrees(math.atan2(dE, dN))   # world frame
        heading_deg    = math.degrees(self.psi_ned)         # world frame
        # error angle: body frame 기준 (-180~180)
        error_angle_deg = (goal_angle_deg - heading_deg + 540.0) % 360.0 - 180.0
        distance_to_goal = math.hypot(dx, dy)

        # 도달 처리
        if distance_to_goal < self.goal_tolerance:
            self.reverse_and_rotate_in_place()
            return

        # 제어 및 퍼블리시 (상태머신 내장)
        self.calculate_thruster(error_angle_deg, distance_to_goal)
        self.control_publish()

        # 시각화
        self.visualize_state(wp_goal=wp_goal)

    # -------------------- RViz --------------------
    def visualize_state(self, gate_mid=None, wp_goal=None, use_deleteall=True):
        ma = MarkerArray()
        if use_deleteall:
            mdel = Marker(); mdel.action = Marker.DELETEALL
            ma.markers.append(mdel)

        mid = 0

        # Ship (blue sphere)
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
        m.ns = "ship"; m.id = mid; mid += 1
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = self.x_ned; m.pose.position.y = self.y_ned; m.pose.position.z = 0.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 1.0
        ma.markers.append(m)

        # Ship heading (arrow)
        a = Marker()
        a.header.frame_id = "map"; a.header.stamp = rospy.Time.now()
        a.ns = "ship_heading"; a.id = 9999
        a.type = Marker.ARROW; a.action = Marker.ADD
        a.points = [
            Point(x=self.x_ned, y=self.y_ned, z=0.0),
            Point(x=self.x_ned + 2.0 * math.cos(self.psi_ned),
                  y=self.y_ned + 2.0 * math.sin(self.psi_ned),
                  z=0.0)
        ]
        a.scale.x = 0.1
        a.scale.y = 0.2
        a.scale.z = 0.3
        a.color.r, a.color.g, a.color.b, a.color.a = 0.0, 0.0, 1.0, 1.0
        ma.markers.append(a)

        # Obstacles (red spheres)
        for obs in self.obstacles:
            c, s = math.cos(self.psi_ned), math.sin(self.psi_ned)
            bx, by = obs.center.x, obs.center.y
            ox = self.x_ned + c * bx - s * by
            oy = self.y_ned + s * bx + c * by

            m = Marker()
            m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
            m.ns = "obstacles"; m.id = mid; mid += 1
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = ox; m.pose.position.y = oy; m.pose.position.z = 0.0
            size = max(float(getattr(obs, "radius", 0.15)) * 2.0, 0.2)
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.7
            ma.markers.append(m)

        # Waypoint (yellow sphere)
        if wp_goal is not None:
            m = Marker()
            m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
            m.ns = "waypoint"; m.id = 10001
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(wp_goal[0])
            m.pose.position.y = float(wp_goal[1])
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            ma.markers.append(m)

            # Ship -> Waypoint arrow (blue)
            a = Marker()
            a.header.frame_id = "map"; a.header.stamp = rospy.Time.now()
            a.ns = "ship_to_wp"; a.id = 10002
            a.type = Marker.ARROW; a.action = Marker.ADD
            a.points = [Point(x=self.x_ned, y=self.y_ned, z=0.0),
                        Point(x=float(wp_goal[0]), y=float(wp_goal[1]), z=0.0)]
            a.scale.x = 0.05; a.scale.y = 0.10
            a.color.r, a.color.g, a.color.b, a.color.a = 0.0, 0.4, 1.0, 0.9
            ma.markers.append(a)

        # Optimal angle arrow (green)
        if hasattr(self, "prev_optimal"):
            length = 5.0
            world_opt_deg = (math.degrees(self.psi_ned) + float(self.prev_optimal))
            ang = math.radians(world_opt_deg)
            tx = self.x_ned + length * math.cos(ang)
            ty = self.y_ned + length * math.sin(ang)

            m = Marker()
            m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
            m.ns = "optimal_angle"; m.id = 20001
            m.type = Marker.ARROW; m.action = Marker.ADD
            m.points = [Point(x=self.x_ned, y=self.y_ned, z=0.0),
                        Point(x=tx, y=ty, z=0.0)]
            m.scale.x = 0.06; m.scale.y = 0.18
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)


# -------------------- main --------------------
def main():
    rospy.init_node("ship_safe_angle_navigation", anonymous=True)
    rate_hz = rospy.get_param("~rate_hz", 10)
    rate = rospy.Rate(rate_hz)
    ship = SHIP()
    while not rospy.is_shutdown():
        ship.detection_run()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
