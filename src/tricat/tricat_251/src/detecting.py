#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import UInt16, Int32MultiArray
from geometry_msgs.msg import PointStamped, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from tricat_msgs.msg import Pose, Control

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (Buffer.transform에서 사용)

D2R = math.pi / 180.0

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class TrackerController:
    """
    - /cluster_0..5 : 각 클러스터 포인트클라우드(PointCloud2)
    - /obj_id       : 길이 6 배열 (KF#i -> 클러스터 인덱스 매칭)
    - /Pose         : 선체의 위치/자세 (x,y,psi)
    - 출력:
      /Control (thruster_p, thruster_s)
      /visualization_marker_array
      TF: map -> base_link
    """

    def __init__(self):
        # ===== 파라미터 =====
        self.base_thrust    = float(rospy.get_param("~base_thrust", 1550))
        self.thrust_min     = float(rospy.get_param("~thrust_min", 1100))
        self.thrust_max     = float(rospy.get_param("~thrust_max", 1900))
        self.kp_thruster    = float(rospy.get_param("~kp_thruster", 4.0))
        self.perp_dir       = int(rospy.get_param("~perp_dir", 1))   # +1: 좌측직교, -1: 우측직교

        # ★ sticky lock 관련
        self.follow_obj_id  = int(rospy.get_param("~follow_obj_id", 0))  # /obj_id[여기]만 추종
        self.miss_limit     = int(rospy.get_param("~miss_limit", 5))     # 연속 miss 허용
        self.print_every    = float(rospy.get_param("~print_every", 0.5))# 로그 토글(초)

        # ===== 상태 =====
        self.x_map = 0.0
        self.y_map = 0.0
        self.psi_map = 0.0
        self.pose_received = False

        # 클러스터 센트로이드 (map, base_link)
        self.cluster_centroids_map = {i: None for i in range(6)}
        self.cluster_centroids_bl  = {i: None for i in range(6)}

        # /obj_id 원본값
        self.obj_ids = []

        # ★ sticky 잠금 상태
        self.current_lock_idx = None   # 현재 잠금 중인 "클러스터 인덱스"
        self.miss_cnt = 0              # 연속 miss 카운터
        self.lock_cp_bl = None         # 잠금 타깃의 base_link 좌표

        # 제어 관련
        self.seg_vec = None
        self.perp_vec = None
        self.control_angle = None
        self.thruster_p = self.base_thrust
        self.thruster_s = self.base_thrust

        # ===== ROS I/O =====
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.marker_pub  = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 구독
        rospy.Subscriber("/Pose", Pose, self.pose_cb, queue_size=1)
        rospy.Subscriber("/obj_id", Int32MultiArray, self.obj_id_cb, queue_size=1)

        self.cluster_subs = []
        for i in range(6):
            topic = f"/cluster_{i}"
            self.cluster_subs.append(
                rospy.Subscriber(
                    topic,
                    __import__("sensor_msgs").msg.PointCloud2,  # 타입 직접 지정
                    lambda msg, idx=i: self.cluster_cb(msg, idx),
                    queue_size=1
                )
            )

    # ===== 콜백 =====
    def pose_cb(self, msg: Pose):
        self.x_map  = float(msg.x.data)
        self.y_map  = float(msg.y.data)
        self.psi_map = wrap_pi(float(msg.psi.data) * D2R)
        self.pose_received = True
        self.publish_tf()

    def obj_id_cb(self, msg: Int32MultiArray):
        self.obj_ids = list(msg.data)

    def cluster_cb(self, cloud_msg, idx):
        sumx, sumy, n = 0.0, 0.0, 0
        for p in pc2.read_points(cloud_msg, field_names=("x", "y"), skip_nans=True):
            sumx += p[0]; sumy += p[1]; n += 1

        if n == 0:
            self.cluster_centroids_map[idx] = None
            self.cluster_centroids_bl[idx] = None
            return

        cx = sumx / n
        cy = sumy / n
        self.cluster_centroids_map[idx] = (cx, cy)

        ps = PointStamped()
        ps.header = cloud_msg.header  # 보통 frame_id="map"
        ps.point.x, ps.point.y, ps.point.z = cx, cy, 0.0

        try:
            ps_bl = self.tf_buffer.transform(ps, "base_link", rospy.Duration(0.05))
            self.cluster_centroids_bl[idx] = (ps_bl.point.x, ps_bl.point.y)
        except Exception:
            self.cluster_centroids_bl[idx] = None

    # ===== TF 브로드캐스트 =====
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x_map
        t.transform.translation.y = self.y_map
        t.transform.translation.z = 0.0
        cz = math.cos(self.psi_map * 0.5)
        sz = math.sin(self.psi_map * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sz
        t.transform.rotation.w = cz
        self.tf_broadcaster.sendTransform(t)

    # ===== Sticky Lock 로직 =====
    def _visible(self, idx: int) -> bool:
        """해당 클러스터가 현재 프레임에서 센트로이드가 들어왔는지"""
        if idx is None or idx < 0 or idx > 5:
            return False
        return (self.cluster_centroids_bl.get(idx) is not None)

    def _want_idx(self):
        """원본 /obj_id에서 내가 따라야 할 인덱스(클러스터 번호)를 꺼냄"""
        if len(self.obj_ids) >= (self.follow_obj_id + 1):
            return self.obj_ids[self.follow_obj_id]
        return None

    def update_sticky_lock(self):
        """
        - 현재 락이 보이면: 유지(miss_cnt=0)
        - 현재 락이 안 보이면: miss_cnt+=1; miss_limit 초과 시에만 새 want로 갈아탐
        - 현재 락이 없으면: want가 보이면 바로 잠금
        """
        want = self._want_idx()

        if self.current_lock_idx is None:
            if self._visible(want):
                self.current_lock_idx = want
                self.miss_cnt = 0
            # 못보면 대기
            return

        # 현재 락이 있을 때
        if self._visible(self.current_lock_idx):
            self.miss_cnt = 0  # 유지
        else:
            self.miss_cnt += 1
            if self.miss_cnt > self.miss_limit:
                # 갈아타기 시도 (새 want가 보일 때만)
                if self._visible(want):
                    self.current_lock_idx = want
                    self.miss_cnt = 0
                # 새 want도 안 보이면 그대로 유지(다음 프레임에 재시도)

        # 잠금 좌표 갱신
        self.lock_cp_bl = (
            None if not self._visible(self.current_lock_idx)
            else self.cluster_centroids_bl[self.current_lock_idx]
        )

    # ===== 제어 파이프라인 =====
    def build_segment(self):
        if self.lock_cp_bl is None:
            self.seg_vec = None
            return
        dx, dy = self.lock_cp_bl
        if math.hypot(dx, dy) < 1e-6:
            self.seg_vec = None
        else:
            self.seg_vec = (dx, dy)

    def build_perpendicular(self):
        if self.seg_vec is None:
            self.perp_vec = None
            return
        dx, dy = self.seg_vec
        if self.perp_dir >= 0:
            px, py = -dy, dx   # 좌측직교
        else:
            px, py = dy, -dx   # 우측직교
        n = math.hypot(px, py)
        self.perp_vec = None if n < 1e-9 else (px/n, py/n)

    def compute_control(self):
        if self.perp_vec is None:
            self.control_angle = None
            return
        dx, dy = self.perp_vec
        theta = math.atan2(dy, dx)
        self.control_angle = wrap_pi(theta)

        e_deg = math.degrees(self.control_angle)
        diff = self.kp_thruster * e_deg
        left  = self.base_thrust + diff
        right = self.base_thrust - diff
        self.thruster_p = float(max(min(left,  self.thrust_max), self.thrust_min))
        self.thruster_s = float(max(min(right, self.thrust_max), self.thrust_min))

    def publish_control(self):
        msg = Control()
        msg.thruster_p = UInt16(int(self.thruster_p))
        msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(msg)

    def publish_markers(self):
        ma = MarkerArray()
        mid = 0

        # 잠금 점
        if self.lock_cp_bl is not None:
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "locked_cluster"
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.x = self.lock_cp_bl[0]
            m.pose.position.y = self.lock_cp_bl[1]
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
            ma.markers.append(m)

        # 선체 heading (x축)
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "heading"
        m.id = mid; mid += 1
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.2, 1.0, 1.0
        m.points.append(Point(0.0, 0.0, 0.0))
        m.points.append(Point(1.0, 0.0, 0.0))
        ma.markers.append(m)

        # 직교 벡터
        if self.perp_vec is not None:
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "perpendicular"
            m.id = mid; mid += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 1.0
            m.points.append(Point(0.0, 0.0, 0.0))
            m.points.append(Point(self.perp_vec[0], self.perp_vec[1], 0.0))
            ma.markers.append(m)

        # 스러스터 텍스트
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "thrusters"
        m.id = mid; mid += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = 0.5
        m.pose.position.y = 0.0
        m.pose.position.z = 0.5
        m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0
        m.text = f"P={int(self.thruster_p)}  S={int(self.thruster_s)}"
        ma.markers.append(m)

        self.marker_pub.publish(ma)

    # ===== 루프 =====
    def step(self):
        if not self.pose_received:
            return

        # 1) sticky lock 업데이트 (/obj_id[follow_obj_id]만 추종)
        self.update_sticky_lock()

        # 2) 타깃 벡터/직교 벡터
        self.build_segment()
        self.build_perpendicular()

        # 3) 제어
        self.compute_control()

        # 4) 퍼블리시
        if self.control_angle is not None:
            self.publish_control()
        self.publish_markers()

        # 로그 (과다 출력 방지)
        rospy.loginfo_throttle(
            self.print_every,
            "[CTRL] follow_obj_id=%s  lock_idx=%s  miss=%s  lock_cp_bl=%s  "
            "seg=%s  perp=%s  ctrl=%.1fdeg  P=%d S=%d",
            self.follow_obj_id,
            self.current_lock_idx,
            self.miss_cnt,
            None if self.lock_cp_bl is None else "({:.2f},{:.2f})".format(*self.lock_cp_bl),
            None if self.seg_vec is None else "({:.2f},{:.2f})".format(*self.seg_vec),
            None if self.perp_vec is None else "({:.2f},{:.2f})".format(*self.perp_vec),
            0.0 if self.control_angle is None else math.degrees(self.control_angle),
            int(self.thruster_p), int(self.thruster_s)
        )


def main():
    rospy.init_node("cluster_perp_controller", anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    node = TrackerController()
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
