import rospy
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

class Visualizer:
    def __init__(self):
        # 마커 퍼블리셔 초기화
        self.marker_pub = rospy.Publisher("/wall_markers", MarkerArray, queue_size=1)

    def publish_wall_markers(self, front_wall_points, right_wall_points, left_wall_points):
        marker_array = MarkerArray()

        # 정면 벽 마커 추가
        if front_wall_points:
            front_marker = Marker()
            front_marker.header.frame_id = "velodyne"
            front_marker.header.stamp = rospy.Time.now()
            front_marker.ns = "front_wall"
            front_marker.id = 0
            front_marker.type = Marker.LINE_STRIP
            front_marker.action = Marker.ADD
            front_marker.pose.orientation.w = 1.0
            front_marker.scale.x = 0.1
            front_marker.color.a = 1.0
            front_marker.color.r = 0.0  # 녹색으로 정면 표시
            front_marker.color.g = 1.0  # 녹색 강조
            front_marker.color.b = 0.0  # 파란색 없음
            for point in front_wall_points:
                p = geometry_msgs.msg.Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0
                front_marker.points.append(p)
            marker_array.markers.append(front_marker)

        # 오른쪽 벽 마커 추가
        if right_wall_points:
            right_marker = Marker()
            right_marker.header.frame_id = "velodyne"
            right_marker.header.stamp = rospy.Time.now()
            right_marker.ns = "right_wall"
            right_marker.id = 1
            right_marker.type = Marker.LINE_STRIP
            right_marker.action = Marker.ADD
            right_marker.pose.orientation.w = 1.0
            right_marker.scale.x = 0.1
            right_marker.color.a = 1.0
            right_marker.color.r = 0.0  # 빨간색 없음
            right_marker.color.g = 0.0  # 녹색 없음
            right_marker.color.b = 1.0  # 파란색으로 오른쪽 표시
            for point in right_wall_points:
                p = geometry_msgs.msg.Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0
                right_marker.points.append(p)
            marker_array.markers.append(right_marker)

        # 왼쪽 벽 마커 추가
        if left_wall_points:
            left_marker = Marker()
            left_marker.header.frame_id = "velodyne"
            left_marker.header.stamp = rospy.Time.now()
            left_marker.ns = "left_wall"
            left_marker.id = 2
            left_marker.type = Marker.LINE_STRIP
            left_marker.action = Marker.ADD
            left_marker.pose.orientation.w = 1.0
            left_marker.scale.x = 0.1
            left_marker.color.a = 1.0
            left_marker.color.r = 1.0  # 빨간색 으로 왼쪽 표시
            left_marker.color.g = 0.0  # 녹색 없음
            left_marker.color.b = 0.0  # 파란색 없음
            for point in left_wall_points:
                p = geometry_msgs.msg.Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0
                left_marker.points.append(p)
            marker_array.markers.append(left_marker)

        # 마커 퍼블리시
        self.marker_pub.publish(marker_array)
