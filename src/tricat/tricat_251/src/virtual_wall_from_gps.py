#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
virtual_wall_from_gps.py

- 목적: GPS(위도/경도)로 정의한 "가상 벽(선분)"을 obstacle_detector/Obstacles로 퍼블리시
- 프레임: 기본 "map" (ENU 기준)
- 입력 파라미터(virtual_walls_gps.yaml 참고):
    ~origin_lat (float): ENU 원점 위도
    ~origin_lon (float): ENU 원점 경도
    ~frame_id   (str)  : 퍼블리시 frame_id (기본 "map")
    ~pub_topic  (str)  : 퍼블리시 토픽 (기본 "/obstacles_virtual")
    ~publish_rate (float, Hz)
    ~marker_line_thickness (float): RViz 선 두께(미터)
    ~walls_gps: [ [[lat1,lon1],[lat2,lon2]], ... ]  # 두 점으로 만든 직선 벽(여러 개)
    ~polylines_gps: [ [[latA,lonA],[latB,lonB],...], ... ] # 연속 점을 인접 연결

- 주의: 아래 ENU 변환은 소구역(수 km 이하)에서 충분한 근사 정확도를 제공합니다.
"""

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_detector.msg import Obstacles, SegmentObstacle

R_EARTH = 6378137.0  # WGS-84 (m)

def deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def ll_to_enu(lat: float, lon: float, lat0: float, lon0: float):
    """
    위경도(lat, lon)를 기준점(lat0, lon0) 대비 ENU (East, North) [m]로 근사 변환.
    - lat/lon 단위: degree
    - 반환: (x_east[m], y_north[m])
    """
    lat_r  = deg2rad(lat)
    lon_r  = deg2rad(lon)
    lat0_r = deg2rad(lat0)
    lon0_r = deg2rad(lon0)
    dlon = lon_r - lon0_r
    dlat = lat_r - lat0_r
    x_east  = R_EARTH * math.cos(lat0_r) * dlon
    y_north = R_EARTH * dlat
    return x_east, y_north

class VirtualWallFromGPS:
    def __init__(self):
        rospy.init_node("virtual_wall_from_gps")

        # === 파라미터 ===
        if not rospy.has_param("~origin_lat") or not rospy.has_param("~origin_lon"):
            rospy.logfatal("~origin_lat / ~origin_lon 파라미터가 필요합니다.")
            raise rospy.ROSInitException("Missing origin_lat/origin_lon")

        self.origin_lat = float(rospy.get_param("~origin_lat"))
        self.origin_lon = float(rospy.get_param("~origin_lon"))
        self.frame_id   = rospy.get_param("~frame_id", "map")
        self.pub_topic  = rospy.get_param("~pub_topic", "/obstacles_virtual")
        self.rate_hz    = float(rospy.get_param("~publish_rate", 10.0))
        self.line_thick = float(rospy.get_param("~marker_line_thickness", 0.06))

        # GPS 입력
        self.walls_gps = rospy.get_param("~walls_gps", [])
        self.polylines_gps = rospy.get_param("~polylines_gps", [])

        # 퍼블리셔
        self.pub_obs = rospy.Publisher(self.pub_topic, Obstacles, queue_size=10)
        self.pub_marker = rospy.Publisher("~markers", MarkerArray, queue_size=10)

        # ENU 선분 사전 변환
        self.segments_enu = self._build_segments()

        rospy.loginfo("[virtual_wall_from_gps] origin=(%.7f, %.7f), frame_id=%s, segments=%d",
                      self.origin_lat, self.origin_lon, self.frame_id, len(self.segments_enu))

    def _build_segments(self):
        segs = []

        # 1) 두 점으로 된 벽들
        for idx, pair in enumerate(self.walls_gps):
            if not isinstance(pair, list) or len(pair) != 2:
                rospy.logwarn("walls_gps[%d]는 [[lat1,lon1],[lat2,lon2]] 형식이어야 합니다: %s", idx, str(pair))
                continue
            try:
                (lat1, lon1), (lat2, lon2) = pair
                x1, y1 = ll_to_enu(float(lat1), float(lon1), self.origin_lat, self.origin_lon)
                x2, y2 = ll_to_enu(float(lat2), float(lon2), self.origin_lat, self.origin_lon)
                segs.append((x1, y1, x2, y2))
            except Exception as e:
                rospy.logwarn("walls_gps[%d] 파싱 실패: %s", idx, str(e))
                continue

        # 2) 폴리라인(연속 점 연결)
        for pidx, poly in enumerate(self.polylines_gps):
            if not isinstance(poly, list) or len(poly) < 2:
                continue
            for i in range(len(poly) - 1):
                try:
                    lat1, lon1 = poly[i]
                    lat2, lon2 = poly[i + 1]
                    x1, y1 = ll_to_enu(float(lat1), float(lon1), self.origin_lat, self.origin_lon)
                    x2, y2 = ll_to_enu(float(lat2), float(lon2), self.origin_lat, self.origin_lon)
                    segs.append((x1, y1, x2, y2))
                except Exception as e:
                    rospy.logwarn("polylines_gps[%d][%d] 파싱 실패: %s", pidx, i, str(e))
                    continue

        return segs

    def make_obstacles(self) -> Obstacles:
        h = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        msg = Obstacles()
        msg.header = h

        for (x1, y1, x2, y2) in self.segments_enu:
            seg = SegmentObstacle()
            seg.first_point = Point(x=x1, y=y1, z=0.0)
            seg.last_point  = Point(x=x2, y=y2, z=0.0)
            seg.velocity    = Vector3(x=0.0, y=0.0, z=0.0)
            msg.segments.append(seg)

        return msg

    def make_markers(self) -> MarkerArray:
        ma = MarkerArray()
        now = rospy.Time.now()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = "virtual_walls_gps"
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = self.line_thick
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 0.3
        line.color.a = 0.9
        line.lifetime = rospy.Duration(0.5)

        for (x1, y1, x2, y2) in self.segments_enu:
            line.points.append(Point(x=x1, y=y1, z=0.0))
            line.points.append(Point(x=x2, y=y2, z=0.0))

        ma.markers.append(line)
        return ma

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub_obs.publish(self.make_obstacles())
            self.pub_marker.publish(self.make_markers())
            rate.sleep()

if __name__ == "__main__":
    try:
        VirtualWallFromGPS().spin()
    except rospy.ROSInterruptException:
        pass
