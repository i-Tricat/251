#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import math
import numpy as np
from itertools import combinations
from std_msgs.msg import UInt16
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from ship.wp_manager import WpManager
import tf2_ros

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

COL = {
    "reset":  "\033[0m","bold": "\033[1m","red": "\033[31m","green":"\033[32m",
    "yellow": "\033[33m","blue":"\033[34m","mag":"\033[35m","cyan":"\033[36m","gray":"\033[90m",
}
def color(txt, c="reset"): return f"{COL.get(c,'')}{txt}{COL['reset']}"

class SHIP:
    def __init__(self):
        # === Waypoint(큰 목표) ===
        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_k = []; self.d_goal = 0.0
        if getattr(self.wp_manager, "WP_data", None):
            try: self.wp_manager.initialize()
            except Exception: pass

        # === Pose ===
        self.x_ned = 0.0; self.y_ned = 0.0; self.psi_ned = 0.0; self.r_ned = 0.0
        self.pose_received = False
        rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)

        # === LiDAR / Obstacles ===
        rospy.Subscriber("/obstacles", Obstacles, self.obstacles_callback, queue_size=10)
        self.obstacles = []  # BODY frame centers, inflated radius

        # 좌표계/TF
        self.obstacles_frame = rospy.get_param("~obstacles_frame", "scanner")  # "scanner" | "world"
        self.map_frame  = rospy.get_param("~map_frame",  "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # LiDAR 파라미터
        self.lidar_min_exclusion  = rospy.get_param("~lidar_min_exclusion", 0.02)  # [m]
        self.lidar_max_range      = rospy.get_param("~lidar_max_range", 3.0)       # [m]
        self.lidar_yaw_range      = rospy.get_param("~lidar_yaw_range", 180.0)     # [deg]
        self.lidar_margin         = rospy.get_param("~lidar_margin", 0.40)         # [m]
        self.lidar_distance_range = rospy.get_param("~lidar_distance_range", 3.0)  # [m]

        # === Gate → micro-WP 설정 ===
        self.min_gate_width     = rospy.get_param("~min_gate_width", 1.5)      # [m] 통과 최소 폭
        self.gate_angle_tol_deg = rospy.get_param("~gate_angle_tol_deg", 10.0) # [deg] 목표선과 허용 각도차(좌 우 따로 25 25)
        self.gate_forward_only  = rospy.get_param("~gate_forward_only", True)  # 전방만 사용

        # micro-WP(짧은 웨이포인트) 파라미터
        self.micro_wp_lookahead = rospy.get_param("~micro_wp_lookahead", 0.8)      # [m] 게이트 중심에서 전방으로
        self.micro_wp_radius    = rospy.get_param("~micro_wp_radius",   0.5)       # [m] 도달 판정 반경
        self.micro_wp_min_spacing = rospy.get_param("~micro_wp_min_spacing", 0.3)  # [m] 연1 생성 최소 간격

        # === Control ===
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.control_msg = Control()

        self.neutral_thrust = rospy.get_param("~neutral_thrust", 1500)  # 정지
        self.base_thrust    = rospy.get_param("~base_thrust",   1600)   # 약한 전진 바이어스
        self.use_base_bias  = rospy.get_param("~use_base_bias", True)   # 살짝 굴리는 용도

        self.thrust_range = rospy.get_param("~thrust_range", [1300, 1700])
        self.kp_thruster  = rospy.get_param("~kp_thruster", 2.2)
        self.kd_thruster  = rospy.get_param("~kd_thruster", 0.12)  # dt 반영s
        self.max_pd       = rospy.get_param("~max_pd", 220.0)
        self.deadzone_deg = rospy.get_param("~deadzone_deg", 0.0)

        self.thruster_p = self.neutral_thrust
        self.thruster_s = self.neutral_thrust
        self.prev_optimal = 0.0
        self._last_t = rospy.Time.now()

        # === 가상 짧은 웨이포인트 상태 ===
        # micro_wp: dict {"x": map_x, "y": map_y, "t": rospy.Time} 또는 None
        self.micro_wp = None
        self.last_micro_wp_xy = None  # 최근 생성 위치(간격 보장용)

        # 시각화/디버깅용
        self.last_chosen_gate = None  # BODY frame 저장

        # === RViz ===
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)

    # ---------- Pose / TF ----------
    def pose_callback(self, msg):
        self.x_ned = float(msg.x.data); self.y_ned = float(msg.y.data)
        self.psi_ned = wrap_pi(float(msg.psi.data) * D2R)
        self.r_ned = float(msg.r.data) if hasattr(msg, "r") else 0.0
        self.pose_received = True

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame; t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x_ned; t.transform.translation.y = self.y_ned
        cy = math.cos(self.psi_ned/2.0); sy = math.sin(self.psi_ned/2.0)
        t.transform.rotation.z = sy; t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

    # ---------- Waypoints ----------
    def update_waypoints(self):
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)

    def current_waypoint(self):
        if len(self.WP_k) >= 2:
            return np.array([self.WP_k[1].x.data, self.WP_k[1].y.data])
        return None

    # ---------- Obstacles ----------
    def obstacles_callback(self, msg):
        if not msg.circles:
            self.obstacles = []; return
        fov_half = math.radians(self.lidar_yaw_range)/2.0
        kept = []
        for c in msg.circles:
            if self.obstacles_frame == "scanner":
                bx, by = -c.center.x, -c.center.y  # BODY
                d = math.hypot(bx, by); bearing = math.atan2(by, bx)
            else:
                dx, dy = c.center.x - self.x_ned, c.center.y - self.y_ned
                cosh, sinh = math.cos(-self.psi_ned), math.sin(-self.psi_ned)
                bx = cosh*dx - sinh*dy; by = sinh*dx + cosh*dy
                d = math.hypot(bx, by); bearing = math.atan2(by, bx)
            if d < self.lidar_min_exclusion or d > self.lidar_max_range: continue
            if abs(bearing) > fov_half: continue
            c.radius = float(getattr(c,"radius",0.0)) + self.lidar_margin
            c.center.x, c.center.y = bx, by
            kept.append(c)
        self.obstacles = kept

    # ---------- Gates ----------
    def make_gates(self):
        gates = []
        obs = [o for o in self.obstacles if math.hypot(o.center.x, o.center.y) <= self.lidar_distance_range]
        if len(obs) < 2: return gates
        fov_half = math.radians(self.lidar_yaw_range)/2.0

        for a, b in combinations(obs, 2):
            ax, ay, ar = a.center.x, a.center.y, float(getattr(a,"radius",0.15))
            bx, by, br = b.center.x, b.center.y, float(getattr(b,"radius",0.15))
            vx, vy = (bx-ax), (by-ay); d = math.hypot(vx, vy)
            if d < 1e-6: continue
            width = d - (ar + br)
            if width < self.min_gate_width: continue
            ux, uy = (vx/d), (vy/d)
            p1x, p1y = ax + ar*ux, ay + ar*uy
            p2x, p2y = bx - br*ux, by - br*uy
            cx, cy = (p1x+p2x)*0.5, (p1y+p2y)*0.5
            ang = math.atan2(cy, cx); dist = math.hypot(cx, cy)
            if self.gate_forward_only and abs(ang) > fov_half: continue
            if dist > self.lidar_distance_range: continue
            gates.append({"cx":cx,"cy":cy,"width":width,"ang":ang,"dist":dist})
        return gates

    def select_gate_toward_goal(self, gates, desired_body_angle_deg):
        if not gates: return None
        tol = abs(self.gate_angle_tol_deg)
        cand = []
        for g in gates:
            gate_deg = math.degrees(g["ang"])
            ang_diff = abs((gate_deg - desired_body_angle_deg + 540.0)%360.0 - 180.0)
            if ang_diff <= tol:
                cand.append((g, ang_diff, g["width"], g["dist"]))
        if not cand: return None
        # 우선순위: 각도차 최소 → 폭 최대 → 거리 최소
        cand.sort(key=lambda t: (t[1], -t[2], t[3]))
        return cand[0][0]

    # ---------- micro-WP 생성/관리 ----------
    def create_micro_wp_from_gate(self, gate_body):
        """게이트 BODY 좌표를 MAP으로 변환하고, 진행방향으로 lookahead만큼 더 밀어 micro-WP 생성"""
        # BODY → MAP
        c, s = math.cos(self.psi_ned), math.sin(self.psi_ned)
        gx_map = self.x_ned + c*gate_body["cx"] - s*gate_body["cy"]
        gy_map = self.y_ned + s*gate_body["cx"] + c*gate_body["cy"]

        # 게이트 중심 방향(맵 기준) 벡터
        vx = gx_map - self.x_ned; vy = gy_map - self.y_ned
        vnorm = math.hypot(vx, vy)
        if vnorm < 1e-6:
            tx, ty = gx_map, gy_map
        else:
            ux, uy = vx/vnorm, vy/vnorm
            tx, ty = gx_map + ux*self.micro_wp_lookahead, gy_map + uy*self.micro_wp_lookahead

        # 최소 간격 보장
        if self.last_micro_wp_xy is not None:
            lx, ly = self.last_micro_wp_xy
            if math.hypot(tx-lx, ty-ly) < self.micro_wp_min_spacing:
                # 간격이 너무 짧으면 게이트 중심만 사용
                tx, ty = gx_map, gy_map

        self.micro_wp = {"x": tx, "y": ty, "t": rospy.Time.now()}
        self.last_micro_wp_xy = (tx, ty)

        rospy.loginfo(
            f"{color('[MICRO-WP NEW]', 'green')} "
            f"{color('at', 'gray')}=({tx:.2f},{ty:.2f})  "
            f"{color('from_gate', 'cyan')}=({gx_map:.2f},{gy_map:.2f})  "
            f"{color('lookahead', 'yellow')}={self.micro_wp_lookahead:.2f} m"
        )

    def micro_wp_reached(self):
        if self.micro_wp is None: return False
        dx = self.micro_wp["x"] - self.x_ned
        dy = self.micro_wp["y"] - self.y_ned
        return math.hypot(dx, dy) <= self.micro_wp_radius

    # ---------- Control ----------
    def calculate_thruster_to_point(self, target_map_xy):
        # map 목표점 → body 각오차
        dx = target_map_xy[0] - self.x_ned
        dy = target_map_xy[1] - self.y_ned
        goal_angle_deg = math.degrees(math.atan2(dy, dx))      # world
        heading_deg    = math.degrees(self.psi_ned)            # world
        err_deg = (goal_angle_deg - heading_deg + 540.0)%360.0 - 180.0

        # PD
        now_t = rospy.Time.now()
        dt = max(1e-3, (now_t - self._last_t).to_sec())
        delta = err_deg - self.prev_optimal
        if abs(err_deg) < self.deadzone_deg:
            err_deg = 0.0; delta = 0.0
        d_term = delta / dt
        pd_out = self.kp_thruster * err_deg + self.kd_thruster * d_term
        pd_out = float(np.clip(pd_out, -self.max_pd, self.max_pd))
        self.prev_optimal = err_deg; self._last_t = now_t

        base = self.base_thrust if self.use_base_bias else self.neutral_thrust
        left  = np.clip(base + pd_out, *self.thrust_range)
        right = np.clip(base - pd_out, *self.thrust_range)
        self.thruster_p = int(left); self.thruster_s = int(right)

        rospy.loginfo_throttle(0.5,
            f"\n{color('[MICRO-WP TRACK]', 'bold')} "
            f"{color('tx,ty', 'cyan')}=({target_map_xy[0]:.2f},{target_map_xy[1]:.2f})  "
            f"{color('err', 'yellow')}={err_deg:6.1f}°  "
            f"{color('P', 'blue')}={self.thruster_p:4d}  "
            f"{color('S', 'blue')}={self.thruster_s:4d}"
        )

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    # ---------- Main Loop ----------
    def detection_run(self):
        self.update_waypoints()
        big_wp = self.current_waypoint()
        if big_wp is None:
            # 큰 웨이포인트 없으면 정지
            self.thruster_p = self.neutral_thrust; self.thruster_s = self.neutral_thrust
            self.control_publish(); self.visualize_state()
            rospy.logwarn_throttle(1.0, color("Waypoint 없음 → HOLD", "red"))
            return

        # (A) 이전 micro-WP 도착 체크 → 도착 시 로그 + 큰 웨이포인트까지 거리 출력
        if self.micro_wp is not None and self.micro_wp_reached():
            big_wp_dist = math.hypot(big_wp[0] - self.x_ned, big_wp[1] - self.y_ned)
            rospy.loginfo(
                f"{color('[MICRO-WP ARRIVED]', 'mag')} "
                f"{color('at', 'gray')}=({self.micro_wp['x']:.2f},{self.micro_wp['y']:.2f})  "
                f"{color('dist_to_BIG_WP', 'cyan')}={big_wp_dist:.2f} m  "
                f"{color('radius', 'yellow')}={self.micro_wp_radius:.2f} m"
            )
            self.micro_wp = None  # 소모형

        # (B) micro-WP가 없으면 → 새로운 게이트 선택·micro-WP 생성 시도
        if self.micro_wp is None:
            # big_wp를 향하는 body 각(선호 방향)
            dN = big_wp[0] - self.x_ned; dE = big_wp[1] - self.y_ned
            goal_angle_deg = math.degrees(math.atan2(dE, dN))
            heading_deg    = math.degrees(self.psi_ned)
            desired_body_angle_deg = (goal_angle_deg - heading_deg + 540.0)%360.0 - 180.0

            gates = self.make_gates()
            chosen = self.select_gate_toward_goal(gates, desired_body_angle_deg)
            self.last_chosen_gate = chosen

            if chosen is not None:
                self.create_micro_wp_from_gate(chosen)
            else:
                # 게이트 없으면 big_wp 자체를 향해 유지(느리게 전진/정지)
                self.thruster_p = self.neutral_thrust if not self.use_base_bias else self.base_thrust
                self.thruster_s = self.thruster_p

        # (C) 제어 목표 결정: micro-WP 우선, 없으면 big_wp
        if self.micro_wp is not None:
            target_xy = (self.micro_wp["x"], self.micro_wp["y"])
        else:
            target_xy = (float(big_wp[0]), float(big_wp[1]))

        # 타깃으로 조향
        self.calculate_thruster_to_point(target_xy)
        self.control_publish()
        self.visualize_state(big_wp=big_wp)

    # ---------- Visualization ----------
    def visualize_state(self, big_wp=None, use_deleteall=True):
        ma = MarkerArray()
        if use_deleteall:
            mdel = Marker(); mdel.action = Marker.DELETEALL; ma.markers.append(mdel)
        mid = 0

        # ship
        m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
        m.ns="ship"; m.id=mid; mid+=1; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.x=self.x_ned; m.pose.position.y=self.y_ned
        m.scale.x=m.scale.y=m.scale.z=0.5; m.color.r,m.color.g,m.color.b,m.color.a=0.0,0.0,1.0,1.0
        ma.markers.append(m)

        # heading arrow
        a = Marker(); a.header.frame_id="map"; a.header.stamp=rospy.Time.now()
        a.ns="ship_heading"; a.id=9999; a.type=Marker.ARROW; a.action=Marker.ADD
        a.points=[Point(x=self.x_ned,y=self.y_ned,z=0.0),
                  Point(x=self.x_ned+2.0*math.cos(self.psi_ned),
                        y=self.y_ned+2.0*math.sin(self.psi_ned), z=0.0)]
        a.scale.x=0.1; a.scale.y=0.2; a.scale.z=0.3; a.color.r,a.color.g,a.color.b,a.color.a=0,0,1,1
        ma.markers.append(a)

        # obstacles
        for obs in self.obstacles:
            c, s = math.cos(self.psi_ned), math.sin(self.psi_ned)
            bx, by = obs.center.x, obs.center.y
            ox = self.x_ned + c*bx - s*by; oy = self.y_ned + s*bx + c*by
            m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
            m.ns="obstacles"; m.id=mid; mid+=1; m.type=Marker.SPHERE; m.action=Marker.ADD
            size=max(float(getattr(obs,"radius",0.15))*2.0,0.2)
            m.pose.position.x=ox; m.pose.position.y=oy; m.scale.x=m.scale.y=m.scale.z=size
            m.color.r,m.color.g,m.color.b,m.color.a=1.0,0.0,0.0,0.7; ma.markers.append(m)

        # big waypoint
        if big_wp is not None:
            m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
            m.ns="waypoint"; m.id=10001; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position.x=float(big_wp[0]); m.pose.position.y=float(big_wp[1])
            m.scale.x=m.scale.y=m.scale.z=0.35; m.color.r,m.color.g,m.color.b,m.color.a=1,1,0,1
            ma.markers.append(m)

        # all gate centers (purple)
        gates = self.make_gates(); gid=30000
        for g in gates:
            c, s = math.cos(self.psi_ned), math.sin(self.psi_ned)
            gx = self.x_ned + c*g["cx"] - s*g["cy"]
            gy = self.y_ned + s*g["cx"] + c*g["cy"]
            m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
            m.ns="gates"; m.id=gid; gid+=1; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position.x=gx; m.pose.position.y=gy
            m.scale.x=m.scale.y=m.scale.z=0.25
            m.color.r,m.color.g,m.color.b,m.color.a=0.6,0.0,0.8,0.9
            ma.markers.append(m)

        # chosen gate (lime)
        if self.last_chosen_gate is not None:
            g = self.last_chosen_gate; c, s = math.cos(self.psi_ned), math.sin(self.psi_ned)
            gx = self.x_ned + c*g["cx"] - s*g["cy"]; gy = self.y_ned + s*g["cx"] + c*g["cy"]
            m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
            m.ns="chosen_gate"; m.id=31000; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position.x=gx; m.pose.position.y=gy
            m.scale.x=m.scale.y=m.scale.z=0.40
            m.color.r,m.color.g,m.color.b,m.color.a=0.1,1.0,0.1,1.0
            ma.markers.append(m)

        # micro-WP (cyan sphere + arrow)
        if self.micro_wp is not None:
            mx, my = self.micro_wp["x"], self.micro_wp["y"]
            ms = Marker(); ms.header.frame_id="map"; ms.header.stamp=rospy.Time.now()
            ms.ns="micro_wp"; ms.id=32000; ms.type=Marker.SPHERE; ms.action=Marker.ADD
            ms.pose.position.x=mx; ms.pose.position.y=my
            ms.scale.x=ms.scale.y=ms.scale.z=0.30
            ms.color.r,ms.color.g,ms.color.b,ms.color.a=0.0,1.0,1.0,1.0
            ma.markers.append(ms)

            ar = Marker(); ar.header.frame_id="map"; ar.header.stamp=rospy.Time.now()
            ar.ns="ship_to_micro"; ar.id=32001; ar.type=Marker.ARROW; ar.action=Marker.ADD
            ar.points=[Point(x=self.x_ned,y=self.y_ned,z=0.0), Point(x=mx,y=my,z=0.0)]
            ar.scale.x=0.05; ar.scale.y=0.10
            ar.color.r,ar.color.g,ar.color.b,ar.color.a=0.0,0.8,0.8,0.9
            ma.markers.append(ar)

        # optimal arrow(현재 PD 목표 각)
        if hasattr(self,"prev_optimal"):
            length=5.0; world_opt_deg = (math.degrees(self.psi_ned) + float(self.prev_optimal))
            ang = math.radians(world_opt_deg); tx = self.x_ned + length*math.cos(ang); ty = self.y_ned + length*math.sin(ang)
            m = Marker(); m.header.frame_id="map"; m.header.stamp=rospy.Time.now()
            m.ns="optimal_angle"; m.id=20001; m.type=Marker.ARROW; m.action=Marker.ADD
            m.points=[Point(x=self.x_ned,y=self.y_ned,z=0.0), Point(x=tx,y=ty,z=0.0)]
            m.scale.x=0.06; m.scale.y=0.18; m.color.r,m.color.g,m.color.b,m.color.a=0.0,1.0,0.0,1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)

# -------- main --------
def main():
    rospy.init_node("ship_gate_micro_waypoints", anonymous=True)
    rate = rospy.Rate(10)
    ship = SHIP()
    while not rospy.is_shutdown():
        ship.detection_run()
        rate.sleep()

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
