#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math
import rospy
from std_msgs.msg import UInt16
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

D2R = math.pi / 180.0

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def dist_point_to_segment(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    seg_len2 = vx * vx + vy * vy
    if seg_len2 <= 1e-12:
        d = math.hypot(px - x1, py - y1)
        return d, (x1, y1)
    t = (wx * vx + wy * vy) / seg_len2
    t = max(0.0, min(1.0, t))
    cx, cy = x1 + t * vx, y1 + t * vy
    d = math.hypot(px - cx, py - cy)
    return d, (cx, cy)

class DETECTING:
    def __init__(self):
        self.base_thrust = float(rospy.get_param("~base_thrust", 1550))
        self.thrust_range = list(rospy.get_param("~thrust_range", [1100, 1900]))
        self.kp_thruster = float(rospy.get_param("~kp_thruster", 4.0))
        self.kd_thruster = float(rospy.get_param("~kd_thruster", 0.3))
        self.perp_dir = int(rospy.get_param("~perp_dir", 1))
        self.lock_max_miss = int(rospy.get_param("~lock_max_miss", 15))
        self.lock_gate = float(rospy.get_param("~lock_gate", 2))
        self.lock_priority = int(rospy.get_param("~lock_priority", 1))

        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.r_ned = 0.0
        self.prev_x_ned = 0.0
        self.prev_y_ned = 0.0
        self.pose_received = False

        self.obstacles = []
        self.locked = False
        self.lock_kind = None
        self.lock_cp = None
        self.lock_miss = 0
        self.lock_target = None

        self.control_msg = Control()
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.thruster_p = self.base_thrust
        self.thruster_s = self.base_thrust

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)

        self.seg_vec = None
        self.perp_vec = None
        self.psi_desire = None
        self.control_angle = None

        rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber("/obstacles", Obstacles, self.obstacle_callback, queue_size=10)

    def pose_callback(self, msg: Pose):
        self.x_ned = float(msg.x.data)
        self.y_ned = float(msg.y.data)
        self.psi_ned = wrap_pi(float(msg.psi.data) * D2R)
        try:
            self.r_ned = float(msg.r.data)
        except Exception:
            pass

        self.prev_x_ned = self.x_ned
        self.prev_y_ned = self.y_ned
        self.pose_received = True
        self.publish_tf()

    def obstacle_callback(self, msg: Obstacles):
        self.obstacles = list(msg.circles) + list(msg.segments)

    def _current_candidates(self):
        cands = []
        for c in getattr(self, "obstacles", []):
            if hasattr(c, "center"):
                cx, cy = float(c.center.x), float(c.center.y)
                cands.append(("circle", (cx, cy)))
            elif hasattr(c, "point1") and hasattr(c, "point2"):
                _, (px, py) = dist_point_to_segment(
                    0.0, 0.0,
                    float(c.point1.x), float(c.point1.y),
                    float(c.point2.x), float(c.point2.y)
                )
                cands.append(("segment", (px, py)))
        return cands

    def candidate_similarity(self, cand):
        if self.lock_target is None:
            return 0
        cx, cy = cand[1]
        tx, ty = self.lock_target
        dist = math.hypot(cx - tx, cy - ty)
        angle_c = math.atan2(cy, cx)
        angle_t = math.atan2(ty, tx)
        angle_diff = abs(wrap_pi(angle_c - angle_t))
        return dist + 0.7 * angle_diff

    def acquire_lock(self):
        cands = self._current_candidates()
        if not cands:
            self._release_lock()
            return False

        # 후보 거리 제한
        cands = [c for c in cands if 0.3 < math.hypot(c[1][0], c[1][1]) < 4.0]
        if not cands:
            self._release_lock()
            return False

        # 가장 가까운 장애물 하나만 무조건 lock
        cands.sort(key=lambda c: math.hypot(c[1][0], c[1][1]))
        self.lock_kind, self.lock_cp = cands[0]
        self.lock_target = self.lock_cp
        self.locked = True
        self.lock_miss = 0
        rospy.loginfo(f"[LOCK] kind={self.lock_kind}, dist={math.hypot(*self.lock_cp):.2f} m (NEW TARGET)")
        return True



    def _release_lock(self):
        self.locked = False
        self.lock_kind = None
        self.lock_cp = None
        self.lock_target = None
        self.lock_miss = 0

    def build_segment(self):
        if not self.locked or self.lock_cp is None:
            self.seg_vec = None
            return
        dx = self.lock_cp[0] - self.x_ned
        dy = self.lock_cp[1] - self.y_ned
        dist = math.hypot(dx, dy)
        if dist <= 1e-5:
            self.seg_vec = None
            return
        self.seg_vec = (dx, dy)

    def build_perpendicular(self):
        if self.seg_vec is None:
            self.perp_vec = None
            return
        dx, dy = self.seg_vec
        # 왼쪽 수직벡터: (-dy, dx), 오른쪽: (dy, -dx)
        if self.perp_dir == 1:
            px, py = -dy, dx
        else:
            px, py = dy, -dx
        n = math.hypot(px, py)
        if n > 1e-9:
            self.perp_vec = (px / n, py / n)
        else:
            self.perp_vec = None

    def compute_control_angle(self):
        if self.perp_vec is None:
            self.psi_desire = None
            self.control_angle = None
            return
        dx, dy = self.perp_vec
        self.psi_desire = math.atan2(dy, dx)
        err = wrap_pi(self.psi_desire - self.psi_ned)
        if abs(err) < math.radians(45):
            err = math.copysign(math.radians(45), err)
        self.control_angle = err

    def detection_control(self):
        if self.control_angle is None:
            self.thruster_p = self.base_thrust
            self.thruster_s = self.base_thrust
            return
        e_deg = math.degrees(self.control_angle)
        cp = self.kp_thruster * e_deg
        thrust_diff = cp
        left = self.base_thrust + thrust_diff
        right = self.base_thrust - thrust_diff
        self.thruster_p = max(min(left, self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right, self.thrust_range[1]), self.thrust_range[0])

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    def print_state(self):
        print("===== STATE =====")
        if self.locked and self.lock_cp:
            d = math.hypot(*self.lock_cp)
            print(f"Locked obstacle: {self.lock_kind}, dist={d:.2f} m")
        else:
            print("Locked obstacle: None")
        print(f"psi_ned: {math.degrees(self.psi_ned):.2f} deg")
        if self.psi_desire is not None:
            print(f"psi_desire: {math.degrees(self.psi_desire):.2f} deg")
        if self.control_angle is not None:
            print(f"control_angle: {math.degrees(self.control_angle):.2f} deg")
        print(f"Thruster P={self.thruster_p:.1f}, S={self.thruster_s:.1f}")
        print("=================")

    def detection_run(self):
        if not self.pose_received:
            return
        if not self.obstacles:
            rospy.logwarn_throttle(2.0, "No obstacles yet; waiting...")
            self._release_lock()
            return
        self.acquire_lock()
        self.build_segment()
        self.build_perpendicular()
        self.compute_control_angle()
        self.detection_control()
        self.control_publish()
        self.publish_markers()
        self.print_state()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x_ned
        t.transform.translation.y = self.y_ned
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.psi_ned)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_markers(self):
        ma = MarkerArray()
        mid = 0
        if self.locked and self.lock_cp is not None:
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "locked_obstacle"
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.x = self.lock_cp[0]
            m.pose.position.y = self.lock_cp[1]
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            ma.markers.append(m)

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "ship_heading"
        m.id = mid; mid += 1
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 1.0
        m.points.append(Point(0.0, 0.0, 0.0))
        m.points.append(Point(math.cos(self.psi_ned), math.sin(self.psi_ned), 0.0))
        ma.markers.append(m)

        if self.perp_vec is not None:
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "perp_vec"
            m.id = mid; mid += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            m.points.append(Point(0.0, 0.0, 0.0))
            m.points.append(Point(self.perp_vec[0], self.perp_vec[1], 0.0))
            ma.markers.append(m)

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "thruster_text"
        m.id = mid; mid += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = 0.5
        m.pose.position.y = 0.0
        m.pose.position.z = 0.5
        m.scale.z = 0.3
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0
        m.text = f"P={int(self.thruster_p)}, S={int(self.thruster_s)}"
        ma.markers.append(m)

        self.marker_pub.publish(ma)

def main():
    rospy.init_node("ship_fixed", anonymous=True)
    rate = rospy.Rate(10)
    DETECT = DETECTING()
    while not rospy.is_shutdown():
        DETECT.detection_run()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
