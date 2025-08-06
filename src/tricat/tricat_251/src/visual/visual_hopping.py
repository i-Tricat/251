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