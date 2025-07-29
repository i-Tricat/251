#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import pymap3d as pm  # NED 변환을 위해 사용
import tf.transformations as tft  # 쿼터니언 변환을 위해 추가

class RectangleVisualizer:
    def __init__(self, width=10, length=32, update_rate=1.0):
        self.width = width
        self.length = length

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

        # ROS 파라미터 읽기 with validation
        if not rospy.has_param("origin"):
            rospy.logerr("Parameter 'origin' is not set on the parameter server.")
            rospy.signal_shutdown("Missing 'origin' parameter.")
            return
        self.origin = rospy.get_param("origin")  # [latitude, longitude, altitude]
        if not (isinstance(self.origin, list) and len(self.origin) == 3):
            rospy.logerr("Parameter 'origin' must be a list of three elements: [latitude, longitude, altitude].")
            rospy.signal_shutdown("Invalid 'origin' parameter.")
            return

        if not rospy.has_param("rectangle_fix"):
            rospy.logerr("Parameter 'waypoints' is not set on the parameter server.")
            rospy.signal_shutdown("Missing 'waypoints' parameter.")
            return
        self.gnss_waypoint = rospy.get_param("waypoints")  # 리스트 형태로 가정
        if not isinstance(self.gnss_waypoint, list):
            rospy.logerr("Parameter 'waypoints' must be a list.")
            rospy.signal_shutdown("Invalid 'waypoints' parameter.")
            return

        self.goal_range = rospy.get_param("goal_range", 10)  # 기본값 10m

        # Initialize markers
        self.marker_array = MarkerArray()
        self.init_markers()

        # Timer for periodic updates
        rospy.Timer(rospy.Duration(1.0 / update_rate), self.timer_callback)

    def set_default_pose(self, marker):
        """
        마커의 pose를 기본값으로 설정하는 헬퍼 함수.
        """
        marker.pose.position = Point(0.0, 0.0, 0.0)
        marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    def init_markers(self):
        # Waypoints Marker
        waypoints_marker = Marker()
        waypoints_marker.header.frame_id = "map"
        waypoints_marker.ns = "waypoints"
        waypoints_marker.id = 0
        waypoints_marker.type = Marker.SPHERE_LIST
        waypoints_marker.action = Marker.ADD
        waypoints_marker.scale.x = 0.5
        waypoints_marker.scale.y = 0.5
        waypoints_marker.scale.z = 0.5
        waypoints_marker.color.a = 1.0
        waypoints_marker.color.r = 1.0
        waypoints_marker.color.g = 0.0
        waypoints_marker.color.b = 0.0
        waypoints_marker.lifetime = rospy.Duration(0)  # Indefinite

        # Initialize pose for waypoints_marker
        self.set_default_pose(waypoints_marker)

        # Rectangle Marker
        rectangle_marker = Marker()
        rectangle_marker.header.frame_id = "map"
        rectangle_marker.ns = "rectangle"
        rectangle_marker.id = 1
        rectangle_marker.type = Marker.LINE_STRIP
        rectangle_marker.action = Marker.ADD
        rectangle_marker.scale.x = 0.2
        rectangle_marker.color.a = 1.0
        rectangle_marker.color.r = 0.0
        rectangle_marker.color.g = 0.0
        rectangle_marker.color.b = 1.0
        rectangle_marker.lifetime = rospy.Duration(0)  # Indefinite

        # Initialize pose for rectangle_marker
        self.set_default_pose(rectangle_marker)

        # Center Marker
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.ns = "center"
        center_marker.id = 2
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = 1.0
        center_marker.scale.y = 1.0
        center_marker.scale.z = 1.0
        center_marker.color.a = 1.0
        center_marker.color.r = 0.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.lifetime = rospy.Duration(0)  # Indefinite

        # Initialize pose for center_marker
        center_marker.pose.position = Point(0.0, 0.0, 0.0)
        center_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        # Add markers to MarkerArray
        self.marker_array.markers.append(waypoints_marker)
        self.marker_array.markers.append(rectangle_marker)
        self.marker_array.markers.append(center_marker)

    def get_waypoints(self):
        """
        ROS 파라미터 서버로부터 웨이포인트를 받아옵니다.
        파라미터 이름: ~waypoints
        각 웨이포인트는 [latitude, longitude, altitude] 형식의 리스트여야 합니다.
        """
        waypoints = []
        if not isinstance(self.gnss_waypoint, list):
            rospy.logerr("waypoints 파라미터가 리스트 형식이 아닙니다.")
            return waypoints

        rospy.loginfo(f"Origin: {self.origin}")
        rospy.loginfo(f"Waypoints: {self.gnss_waypoint}")

        for i, waypoint in enumerate(self.gnss_waypoint):
            if not isinstance(waypoint, list) or len(waypoint) < 3:
                rospy.logwarn(f"웨이포인트 {i+1}의 형식이 올바르지 않습니다. [latitude, longitude, altitude] 형식이어야 합니다.")
                continue

            try:
                n, e, _ = self.ned_convert(self.origin, waypoint)
                waypoints.append([n, e])
                rospy.loginfo(f"웨이포인트 {i+1}: North={n}, East={e}")
            except Exception as ex:
                rospy.logerr(f"웨이포인트 {i+1} 변환 실패: {ex}")
                continue

        return waypoints

    def compute_and_publish_markers(self):
        # ROS 파라미터로부터 웨이포인트 다시 읽기
        self.gnss_waypoint = rospy.get_param("rectangle_fix")
        self.points = self.get_waypoints()

        if len(self.points) < 3:
            rospy.logwarn("Insufficient waypoints received. 필요한 웨이포인트 수: 3")
            return

        # 로컬 좌표계의 세 점 정의 (직사각형의 세 모서리)
        A = np.array([
            [-self.width/2, -self.length/2],
            [ self.width/2, -self.length/2],
            [ self.width/2,  self.length/2]
        ])
        B = np.array(self.points[:3])

        # Rigid Transformation 계산
        theta, translation, R_matrix = self.compute_rigid_transform(A, B)

        rospy.loginfo(f"회전 각도 (θ): {theta:.2f} 도")
        rospy.loginfo(f"이동 벡터 (North, East): {translation}")

        # 직사각형 모서리 정의 (로컬 좌표계)
        local_corners = np.array([
            [-self.width/2, -self.length/2],
            [ self.width/2, -self.length/2],
            [ self.width/2,  self.length/2],
            [-self.width/2,  self.length/2],
            [-self.width/2, -self.length/2]  # 직사각형 닫기
        ])

        # 회전 및 이동 적용 (NED 좌표계)
        ned_corners = (R_matrix @ local_corners.T).T + translation

        # Update Waypoints Marker
        waypoints_marker = self.marker_array.markers[0]
        waypoints_marker.header.stamp = rospy.Time.now()
        waypoints_marker.points = []
        for point in B:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # 높이는 0으로 설정
            waypoints_marker.points.append(p)

        # **Pose를 명시적으로 설정**
        self.set_default_pose(waypoints_marker)

        # Update Rectangle Marker
        rectangle_marker = self.marker_array.markers[1]
        rectangle_marker.header.stamp = rospy.Time.now()
        rectangle_marker.points = []
        for corner in ned_corners:
            p = Point()
            p.x = corner[0]
            p.y = corner[1]
            p.z = 0.0  # 높이는 0으로 설정
            rectangle_marker.points.append(p)

        # **Pose를 명시적으로 설정**
        self.set_default_pose(rectangle_marker)

        # Update Center Marker
        center_marker = self.marker_array.markers[2]
        center_marker.header.stamp = rospy.Time.now()
        center_marker.pose.position.x = translation[0]
        center_marker.pose.position.y = translation[1]
        center_marker.pose.position.z = 0.0
        center_marker.pose.orientation = Quaternion(*tft.quaternion_from_euler(0, 0, 0))  # 단위 쿼터니언

        # Publish MarkerArray
        self.marker_pub.publish(self.marker_array)

    def compute_rigid_transform(self, A, B):
        """
        두 점 집합 A와 B 사이의 회전과 이동을 계산.
        A와 B는 (N, 2) 형태의 2D 점 집합.
        """
        assert A.shape == B.shape, "입력된 점들의 형태가 동일해야 합니다."

        # 중심점 계산
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # 중심점 기준으로 이동
        AA = A - centroid_A
        BB = B - centroid_B

        # 공분산 행렬 계산
        H = AA.T @ BB

        # SVD 분해
        U, S, Vt = np.linalg.svd(H)
        R_matrix = Vt.T @ U.T

        # 반사(reflection) 경우 처리
        if np.linalg.det(R_matrix) < 0:
            Vt[1, :] *= -1
            R_matrix = Vt.T @ U.T

        # 회전 각도 계산
        theta = np.degrees(np.arctan2(R_matrix[1, 0], R_matrix[0, 0]))

        # 이동 벡터 계산
        translation = centroid_B - R_matrix @ centroid_A

        return theta, translation, R_matrix

    def ned_convert(self, origin, gnss):
        """
        Geodetic to NED 변환
        :param origin: [latitude, longitude, altitude]
        :param gnss: [latitude, longitude, altitude]
        :return: n, e, d
        """
        # Explicitly specify the ellipsoid
        ellipsoid = pm.Ellipsoid('wgs84')  # Using Ellipsoid object
        # Alternatively, you can pass the string directly if supported
        # ellipsoid = 'wgs84'

        n, e, d = pm.geodetic2ned(
            gnss[0], gnss[1], gnss[2],
            origin[0], origin[1], origin[2],
            ell=ellipsoid, deg=True
        )
        return n, e, d

    def timer_callback(self, event):
        self.compute_and_publish_markers()

def main():
    rospy.init_node("RectangleVisualizer", anonymous=True)
    visualizer = RectangleVisualizer(width=10, length=32, update_rate=1.0)  # 1 Hz 업데이트
    rospy.loginfo("RectangleVisualizer 노드가 시작되었습니다.")

    # 노드가 종료될 때까지 대기
    rospy.spin()

if __name__ == "__main__":
    main()

