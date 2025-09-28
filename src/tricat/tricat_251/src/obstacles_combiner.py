#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obstacles_combiner.py

- 목적: /obstacles_real + /obstacles_virtual -> /obstacles 로 병합 퍼블리시
- 기본 가정: 두 입력 메시지의 frame_id가 동일
- 옵션: ~use_tf:=true, ~output_frame:="map" 등을 주면 프레임이 달라도 TF 변환 후 합침

파라미터:
    ~real_topic      (str)  : 기본 "/obstacles_real"
    ~virtual_topic   (str)  : 기본 "/obstacles_virtual"
    ~output_topic    (str)  : 기본 "/obstacles"
    ~rate            (float): 출력 주기 Hz (기본 10.0)
    ~use_tf          (bool) : 서로 다른 frame일 때 TF 변환 사용(기본 false)
    ~output_frame    (str)  : use_tf가 true일 때 결과를 출력할 frame (예: "map")
"""

import rospy
from obstacle_detector.msg import Obstacles, SegmentObstacle, CircleObstacle

# TF 옵션 사용 시 import
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class ObstaclesCombiner:
    def __init__(self):
        rospy.init_node("obstacles_combiner")

        # 파라미터
        self.real_topic    = rospy.get_param("~real_topic", "/obstacles_real")
        self.virtual_topic = rospy.get_param("~virtual_topic", "/obstacles_virtual")
        self.output_topic  = rospy.get_param("~output_topic", "/obstacles")
        self.rate_hz       = float(rospy.get_param("~rate", 10.0))
        self.use_tf        = bool(rospy.get_param("~use_tf", False))
        self.output_frame  = rospy.get_param("~output_frame", "map")

        # 상태
        self.last_real = None
        self.last_virtual = None

        # 퍼블리셔/서브스크라이버
        self.pub = rospy.Publisher(self.output_topic, Obstacles, queue_size=10)
        rospy.Subscriber(self.real_topic, Obstacles, self.cb_real, queue_size=10)
        rospy.Subscriber(self.virtual_topic, Obstacles, self.cb_virtual, queue_size=10)

        # TF
        if self.use_tf:
            self.tfbuf = tf2_ros.Buffer()
            self.tflis = tf2_ros.TransformListener(self.tfbuf)

        rospy.loginfo("[obstacles_combiner] real=%s, virtual=%s -> output=%s, use_tf=%s, output_frame=%s",
                      self.real_topic, self.virtual_topic, self.output_topic,
                      str(self.use_tf), self.output_frame)

    def cb_real(self, msg: Obstacles):
        self.last_real = msg

    def cb_virtual(self, msg: Obstacles):
        self.last_virtual = msg

    def transform_point(self, pt, from_frame: str, to_frame: str):
        """geometry_msgs/Point -> Point (좌표만 변환; z=0 가정)"""
        ps = PointStamped()
        ps.header.frame_id = from_frame
        ps.header.stamp = rospy.Time(0)  # 최신 TF
        ps.point = pt
        ts = self.tfbuf.transform(ps, to_frame, rospy.Duration(0.2))
        return ts.point

    def transform_obstacles(self, msg: Obstacles, to_frame: str) -> Obstacles:
        """Obstacles(한 프레임) -> to_frame으로 변환한 Obstacles 반환"""
        out = Obstacles()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = to_frame

        # segments
        for seg in msg.segments:
            try:
                p1 = self.transform_point(seg.first_point, msg.header.frame_id, to_frame)
                p2 = self.transform_point(seg.last_point , msg.header.frame_id, to_frame)
            except Exception as e:
                rospy.logwarn("TF segment 변환 실패: %s", str(e))
                continue
            s = SegmentObstacle()
            s.first_point = p1
            s.last_point  = p2
            s.velocity.x = s.velocity.y = s.velocity.z = 0.0
            out.segments.append(s)

        # circles (있다면)
        for c in msg.circles:
            try:
                pc = self.transform_point(c.center, msg.header.frame_id, to_frame)
            except Exception as e:
                rospy.logwarn("TF circle 변환 실패: %s", str(e))
                continue
            cc = CircleObstacle()
            cc.center = pc
            cc.radius = c.radius
            cc.velocity = c.velocity  # 필요 시 별도 처리
            out.circles.append(cc)

        return out

    def merge_msgs_same_frame(self, base: Obstacles, add: Obstacles) -> Obstacles:
        """같은 frame 기준 두 Obstacles를 병합"""
        out = Obstacles()
        out.header = base.header
        out.circles = list(base.circles)
        out.segments = list(base.segments)
        out.circles.extend(add.circles)
        out.segments.extend(add.segments)
        return out

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.last_real is None and self.last_virtual is None:
                rate.sleep()
                continue

            base_msg = self.last_real if self.last_real is not None else self.last_virtual
            out = None

            if not self.use_tf:
                # 프레임 불일치 시 병합 금지(경고) → 보수적으로 base만 퍼블리시
                if self.last_real is not None and self.last_virtual is not None:
                    fr = self.last_real.header.frame_id
                    fv = self.last_virtual.header.frame_id
                    if fr != fv:
                        rospy.logwarn_throttle(2.0,
                            "[obstacles_combiner] frame_id 불일치('%s' vs '%s'). "
                            "use_tf=true + output_frame 설정 사용 권장.", fr, fv)
                        out = base_msg
                    else:
                        out = self.merge_msgs_same_frame(self.last_real, self.last_virtual)
                else:
                    out = base_msg
            else:
                # TF로 둘 다 output_frame으로 변환 후 병합
                msgs = []
                if self.last_real is not None:
                    try:
                        msgs.append(self.transform_obstacles(self.last_real, self.output_frame))
                    except Exception as e:
                        rospy.logwarn("실측 TF 변환 실패: %s", str(e))
                if self.last_virtual is not None:
                    try:
                        msgs.append(self.transform_obstacles(self.last_virtual, self.output_frame))
                    except Exception as e:
                        rospy.logwarn("가상 TF 변환 실패: %s", str(e))

                if len(msgs) == 0:
                    rate.sleep()
                    continue
                out = msgs[0]
                for m in msgs[1:]:
                    out = self.merge_msgs_same_frame(out, m)

            if out is not None:
                out.header.stamp = rospy.Time.now()
                self.pub.publish(out)

            rate.sleep()

if __name__ == "__main__":
    try:
        ObstaclesCombiner().spin()
    except rospy.ROSInterruptException:
        pass
