import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler

def __init__(self):    
    rospy.init_node('Visual_node')
    self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
    self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    self.WP_data = []

def publish_tf(self, event=None):
        current_time = rospy.Time.now()

        # ✅ world → map 변환
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

        # ✅ map → base_link 변환 (보트 위치)
        base_link_transform = TransformStamped()
        base_link_transform.header.stamp = current_time
        base_link_transform.header.frame_id = "map"
        base_link_transform.child_frame_id = "base_link"
        base_link_transform.transform.translation.x = self.x_ned
        base_link_transform.transform.translation.y = self.y_ned
        base_link_transform.transform.translation.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.psi_ned)
        base_link_transform.transform.rotation.x = quaternion[0]
        base_link_transform.transform.rotation.y = quaternion[1]
        base_link_transform.transform.rotation.z = quaternion[2]
        base_link_transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform([map_transform, base_link_transform])
        
def visualize(self, x_ned, y_ned):
            # rospy.loginfo("🚀 visualize 호출됨")
    marker_array = MarkerArray()

    # ✅ 현재 위치 마커 (녹색 구)
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

        ### 웨이포인트 표시 및 도달범위 ###
    for idx, wp in enumerate(self.wp_manager.WP_data):
        # 🚩 웨이포인트 마커 (빨간색 점)
        wp_marker = Marker()
        wp_marker.header.frame_id = "map"
        wp_marker.header.stamp = rospy.Time.now()
        wp_marker.ns = "waypoints"
        wp_marker.id = idx + 100  # ✅ ID 충돌 방지를 위해 고유 값 사용
        wp_marker.type = Marker.SPHERE
        wp_marker.action = Marker.ADD
        wp_marker.pose.position.x = wp.x.data
        wp_marker.pose.position.y = wp.y.data
        wp_marker.pose.position.z = 0
        wp_marker.scale.x = wp_marker.scale.y = wp_marker.scale.z = 0.3

            # 현재 목표 웨이포인트 강조 (노란색)
        if self.WP_k and wp.num.data == self.WP_k[1].num.data:
            wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 1.0, 0.0, 1.0  # 노란색
        else:
            wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 0.0, 0.0, 1.0  # 빨간색

        marker_array.markers.append(wp_marker)

            # 🔴 도달 범위 원 (반투명 빨간색)
        range_marker = Marker()
        range_marker.header.frame_id = "map"
        range_marker.header.stamp = rospy.Time.now()
        range_marker.ns = "waypoint_ranges"
        range_marker.id = idx + 1000  # ✅ 웨이포인트 ID와 겹치지 않도록 충분히 큰 수 사용
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



###### 회피 시각화 더미 #@######

    #  def publish_tf(self, event=None):
#         current_time = rospy.Time.now()

#         # world → map 변환
#         map_transform = TransformStamped()
#         map_transform.header.stamp = current_time
#         map_transform.header.frame_id = "world"
#         map_transform.child_frame_id = "map"
#         map_transform.transform.translation.x = 0.0
#         map_transform.transform.translation.y = 0.0
#         map_transform.transform.translation.z = 0.0
#         map_transform.transform.rotation.x = 0.0
#         map_transform.transform.rotation.y = 0.0
#         map_transform.transform.rotation.z = 0.0
#         map_transform.transform.rotation.w = 1.0

#         # map → base_link 변환 (보트 위치)
#         base_link_transform = TransformStamped()
#         base_link_transform.header.stamp = current_time
#         base_link_transform.header.frame_id = "map"
#         base_link_transform.child_frame_id = "base_link"
#         base_link_transform.transform.translation.x = self.x_ned
#         base_link_transform.transform.translation.y = self.y_ned
#         base_link_transform.transform.translation.z = 0.0
#         quaternion = quaternion_from_euler(0, 0, self.psi_ned)
#         base_link_transform.transform.rotation.x = quaternion[0]
#         base_link_transform.transform.rotation.y = quaternion[1]
#         base_link_transform.transform.rotation.z = quaternion[2]
#         base_link_transform.transform.rotation.w = quaternion[3]

#         # 퍼블리시
#         self.tf_broadcaster.sendTransform([map_transform, base_link_transform])
#         # rospy.loginfo(f"TF 퍼블리시: map→base_link (x={self.x_ned}, y={self.y_ned}, yaw={self.psi_ned})")

#     def publish_scanner_tf(self):
#         t = TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "base_link"   # scanner의 부모 프레임
#         t.child_frame_id = "scanner"      # 생성할 scanner 프레임 이름
#         t.transform.translation.x = 0.0   # 필요 시 값 조정
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0
#         q = quaternion_from_euler(0, 0, 0)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         self.tf_broadcaster.sendTransform(t)
#         # rospy.loginfo(" scanner → base_link 변환 퍼블리시 완료")


#     def visualize(self, visual_detecting_points, non_cross_vector, vector_desired, x_ned, y_ned):
#         # rospy.loginfo("visualize 호출됨")
#         marker_array = MarkerArray()

#         # 현재 위치 마커 (녹색 구)
#         gps_marker = Marker()
#         gps_marker.header.frame_id = "map"
#         gps_marker.header.stamp = rospy.Time.now()
#         gps_marker.ns = "gps_position"
#         gps_marker.id = 0
#         gps_marker.type = Marker.SPHERE
#         gps_marker.action = Marker.ADD
#         gps_marker.pose.position.x = x_ned
#         gps_marker.pose.position.y = y_ned
#         gps_marker.pose.position.z = 0
#         gps_marker.scale.x = gps_marker.scale.y = gps_marker.scale.z = 0.5
#         gps_marker.color.r = 0.0
#         gps_marker.color.g = 0.0
#         gps_marker.color.b = 1.0
#         gps_marker.color.a = 1.0
#         marker_array.markers.append(gps_marker)

#         ### 웨이포인트 표시 및 도달범위 ###
#         for idx, wp in enumerate(self.wp_manager.WP_data):
#             # 🚩 웨이포인트 마커 (빨간색 점)
#             wp_marker = Marker()
#             wp_marker.header.frame_id = "map"
#             wp_marker.header.stamp = rospy.Time.now()
#             wp_marker.ns = "waypoints"
#             wp_marker.id = idx + 100  # ID 충돌 방지를 위해 고유 값 사용
#             wp_marker.type = Marker.SPHERE
#             wp_marker.action = Marker.ADD
#             wp_marker.pose.position.x = wp.x.data
#             wp_marker.pose.position.y = wp.y.data
#             wp_marker.pose.position.z = 0
#             wp_marker.scale.x = wp_marker.scale.y = wp_marker.scale.z = 0.3

#             # 현재 목표 웨이포인트 강조 (노란색)
#             if self.WP_k and wp.num.data == self.WP_k[1].num.data:
#                 wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 1.0, 0.0, 1.0  # 노란색
#             else:
#                 wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 0.0, 0.0, 1.0  # 빨간색

#             marker_array.markers.append(wp_marker)

#             # 도달 범위 원 (반투명 빨간색)
#             range_marker = Marker()
#             range_marker.header.frame_id = "map"
#             range_marker.header.stamp = rospy.Time.now()
#             range_marker.ns = "waypoint_ranges"
#             range_marker.id = idx + 1000  # 웨이포인트 ID와 겹치지 않도록 충분히 큰 수 사용
#             range_marker.type = Marker.CYLINDER
#             range_marker.action = Marker.ADD
#             range_marker.pose.position.x = wp.x.data
#             range_marker.pose.position.y = wp.y.data
#             range_marker.pose.position.z = 0.0
#             range_marker.scale.x = range_marker.scale.y = wp.range.data * 2  # 지름
#             range_marker.scale.z = 0.01
#             range_marker.color.r, range_marker.color.g, range_marker.color.b, range_marker.color.a = 0.5, 0.0, 0.0, 0.3
#             marker_array.markers.append(range_marker)


#             # 벡터 시각화 (화살표 마커)
#         for idx, point in enumerate(visual_detecting_points):
#             arrow_marker = Marker()
#             arrow_marker.header.frame_id = "map"
#             arrow_marker.header.stamp = rospy.Time.now()
#             arrow_marker.ns = "vectors"
#             arrow_marker.id = idx + 1
#             arrow_marker.type = Marker.ARROW
#             arrow_marker.action = Marker.ADD
#             arrow_marker.scale.x = 0.1  # 화살 길이
#             arrow_marker.scale.y = 0.05 # 화살 두께
#             arrow_marker.scale.z = 0.05

#             start_point = Point(x_ned, y_ned, 0)
#             end_point = Point(
#                 x_ned + point[0] * self.range,
#                 y_ned + point[1] * self.range,
#                 0
#             )

#             arrow_marker.points = [start_point, end_point]

#             # 색상 지정
#             if point[2] == vector_desired:
#                 arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 1.0, 0.0, 0.0  # 빨강
#                 arrow_marker.color.a = 1.0
#             elif point[2] in non_cross_vector:
#                 arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 0.0, 0.0, 1.0  # 파랑
#                 arrow_marker.color.a = 0.8
#             else:
#                 arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 0.5, 0.5, 0.5  # 회색
#                 arrow_marker.color.a = 0.4

#             marker_array.markers.append(arrow_marker)


#         # 퍼블리시 및 확인 로그
#         self.marker_array_pub.publish(marker_array)
#         # rospy.loginfo("마커 퍼블리시 완료")