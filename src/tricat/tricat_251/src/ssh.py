#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Float64
from std_msgs.msg import UInt16
from math import sin, cos, radians, degrees, hypot, atan2, pi, sqrt, isnan, isinf
import numpy as np
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from ship.wp_manager import WpManager
from control.autopilot import heading_cal
import math
import cv2
from cv_bridge import CvBridge

##### 시각화를 위한 임포트 #####
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

##### 터미널 색깔 추가 ######
from termcolor import colored
import time

D2R = pi / 180
R2D = 180 / pi

# ===== 부표 감지 관련 함수 (카메라 콜백에서 사용) =====
def apply_white_balance(img):
    if hasattr(cv2, "xphoto") and hasattr(cv2.xphoto, "createSimpleWB"):
        try:
            return cv2.xphoto.createSimpleWB().balanceWhite(img)
        except Exception:
            pass
    f = img.astype(np.float32)
    b, g, r = f.mean(axis=(0,1))
    avg = (b + g + r) / 3.0 + 1e-6
    scale = np.array([avg / max(b, 1e-6), avg / max(g, 1e-6), avg / max(r, 1e-6)], np.float32)
    f *= scale
    return np.clip(f, 0, 255).astype(np.uint8)

def adjust_gamma(image, gamma=1.5):
    inv = 1.0 / max(gamma, 1e-6)
    table = (np.arange(256) / 255.0)**inv * 255.0
    return cv2.LUT(image, table.astype(np.uint8))

def preprocess_image(raw_img, hsv=True):
    img = raw_img.copy()
    img = apply_white_balance(img)
    img = adjust_gamma(img, 1.5)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    if hsv:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img

def define_color_range(hmin,hmax,smin=50,smax=255,vmin=50,vmax=255):
    return np.array([hmin,smin,vmin]), np.array([hmax,smax,vmax])

COLOR_RANGES = {
    'red1': define_color_range(0, 10),
    'red2': define_color_range(170, 180),
    'green': define_color_range(60, 85),
}

def select_color(img_hsv, color_range):
    return cv2.inRange(img_hsv, np.array(color_range[0]), np.array(color_range[1]))

def find_blobs(mask, min_area=600):
    if mask is None or mask.size == 0:
        return []
    
    morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9,9), np.uint8))
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    ret = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = ret[0] if len(ret) == 2 else ret[1]

    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area >= min_area:
            x, y, w, h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            blobs.append(dict(contour=c, area=area, bbox=(x, y, w, h),
                             center=(cx, cy), height=h, width=w))

    blobs.sort(key=lambda b: b['area'], reverse=True)
    return blobs

def bearing_from_px(cx, img_w, hfov_deg=78.0):
    dx = (cx - img_w / 2.0)
    return float(np.deg2rad(hfov_deg) / img_w * dx)

# ===== 통합된 SHIP 클래스 =====
class SHIP:
    def __init__(self):
        rospy.init_node("ship", anonymous=True)

        ### Waypoint ###
        self.wp_manager = WpManager()
        self.wp_manager.wp_client()
        self.WP_data = []
        self.WP_k = []
        self.d_goal = 0
        self.target_heading = 0.0
        self.psi_diff = 0.0
        self.ch = False
        if self.wp_manager.WP_data:
            self.wp_manager.initialize()

        ### Pose ###
        self.x_ned, self.y_ned, self.psi_ned = 0.0, 0.0, 0.0
        self.u_ned, self.v_ned, self.r_ned, self.U = 0.0, 0.0, 0.0, 0.0
        self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)
        self.pose_received = False
        
        ### LiDAR ###
        rospy.Subscriber("/obstacles", Obstacles, self.obstacle_callback, queue_size=10)
        self.range = rospy.get_param("range", 10)
        self.angle_num = rospy.get_param("angle_num", 10)
        self.yaw_range = rospy.get_param("yaw_range", 110)
        self.margin = rospy.get_param("margin", 0.5)
        self.obstacles = []
        self.non_cross_vector_len = 0
        self.safe_vector_pub = rospy.Publisher('/safe_vector', Float64, queue_size=10)
        self.safe_vector_sub = rospy.Subscriber('/safe_vector', Float64, self.safe_vector_cb, queue_size=1)
        self.safe_vector_latest = None

        ### Camera & Buoy Detection ###
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.camera_callback, queue_size=1)
        self.buoy_center_px = None
        self.buoy_target_heading = None # radians
        self.buoy_debug_img = None
        self.image_width = 640
        self.image_height = 480
        self.hfov_deg = rospy.get_param("hfov_deg", 78.0)
        self.min_buoy_area = rospy.get_param("min_area", 800)
        self.buoy_pub_dbg = rospy.Publisher("/buoy/debug_image", Image, queue_size=1)
        self.buoy_err_x_pub = rospy.Publisher("/buoy/err_x", Float64, queue_size=1)

        ### Control ###
        self.control_msg = Control()
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.thruster_s = 0
        self.thruster_p = 0
        self.base_thrust = rospy.get_param("base_thrust", 1500)
        self.thrust_range = rospy.get_param("thrust_range", [1900, 1100] )
        self.kp_thruster = rospy.get_param("kp_thruster", 1.7)
        self.kd_thruster = rospy.get_param("kd_thruster" , 0.3)
        self.arrival_range = rospy.get_param("goal_range", 10)
        
        ### Visualization ###
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

    def pose_callback(self, msg):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)
            psi_deg = float(msg.psi.data)
            self.psi_ned = ((psi_deg * D2R + pi) % (2 * pi)) - pi
            self.pose_received = True
        except Exception as e:
            rospy.logwarn(f"Pose 값 수신 오류: {e}")

    def obstacle_callback(self, msg):
        self.obstacles = msg.circles + msg.segments
    
    def safe_vector_cb(self, msg: Float64):
        try:
            self.safe_vector_latest = float(msg.data)
        except Exception:
            self.safe_vector_latest = None

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            H, W = cv_image.shape[:2]
            self.image_width, self.image_height = W, H
            hsv = preprocess_image(cv_image, hsv=True)

            red_mask = cv2.bitwise_or(
                select_color(hsv, COLOR_RANGES['red1']),
                select_color(hsv, COLOR_RANGES['red2'])
            )
            green_mask = select_color(hsv, COLOR_RANGES['green'])
            
            red_blobs = find_blobs(red_mask, self.min_buoy_area)
            green_blobs = find_blobs(green_mask, self.min_buoy_area)

            R, G = None, None
            if red_blobs: R = red_blobs[0]
            if green_blobs: G = green_blobs[0]
            
            overlay = cv_image.copy()

            if R and G:
                cx = (R['center'][0] + G['center'][0]) // 2
                cy = (R['center'][1] + G['center'][1]) // 2
                self.buoy_center_px = (cx, cy)
                
                # 픽셀 위치를 방위각(heading)으로 변환
                # buoys are relative to camera frame, so add current yaw
                bearing_rad = bearing_from_px(cx, W, self.hfov_deg)
                self.buoy_target_heading = self.psi_ned + bearing_rad

                cv2.circle(overlay, (cx, cy), 6, (255, 255, 255), -1)
                self.buoy_err_x_pub.publish(Float64(cx - W/2.0))
            else:
                self.buoy_center_px = None
                self.buoy_target_heading = None
            
            if R is not None:
                x, y, w, h = R['bbox']
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(overlay, R['center'], 4, (0, 0, 255), -1)
            if G is not None:
                x, y, w, h = G['bbox']
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(overlay, G['center'], 4, (0, 255, 0), -1)

            self.buoy_debug_img = overlay
            self.buoy_pub_dbg.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

        except Exception as e:
            rospy.logerr(f"Camera Callback Error: {e}")

    def ship_run(self):
        # 1. 목표 설정 (부표 > 웨이포인트)
        if self.buoy_target_heading is not None:
            self.target_heading = self.buoy_target_heading
            rospy.loginfo_throttle(1, colored("✅ 부표 목표 추적 중", 'green'))
            self.ch = False
        else:
            self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
            self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)
            self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)
            rospy.loginfo_throttle(1, colored("➡️ 웨이포인트 추적 중", 'yellow'))
            self.ch = True

        # 2. 벡터 기반 장애물 회피
        self.detecting_points = self.make_detecting_vector(self.psi_ned)
        self.non_cross_vector = self.delete_vector_inside_obstacle(self.detecting_points, self.psi_ned, self.x_ned, self.y_ned)
        
        # vector_choose는 target_heading을 기반으로 가장 가까운 안전 벡터를 선택
        self.vector_desired = self.vector_choose(self.non_cross_vector)
        self.safe_vector_pub.publish(Float64(float(self.vector_desired)))

        # 3. PID 제어 및 퍼블리시
        self.thruster_p, self.thruster_s = self.thrust_pid_controller(self.vector_desired, self.psi_ned)
        self.control_publish()

        # 4. 시각화
        self.visualize(self.detecting_points, self.non_cross_vector, self.vector_desired, self.x_ned, self.y_ned)

    def cross_check_segment(self, x1, y1, x2, y2, seg_x1, seg_y1, seg_x2, seg_y2):
        denominator = (seg_x2 - seg_x1) * (y2 - y1) - (seg_y2 - seg_y1) * (x2 - x1)
        if abs(denominator) < 1e-10: return False
        ua = ((seg_y2 - seg_y1) * (x1 - seg_x1) - (seg_x2 - seg_x1) * (y1 - seg_y1)) / denominator
        ub = ((y2 - y1) * (x1 - seg_x1) - (x2 - x1) * (y1 - seg_y1)) / denominator
        if 0 <= ua <= 1 and 0 <= ub <= 1: return True
        return False
    
    def cross_check_circle(self, x1, y1, x2, y2, circle_x, circle_y, radius):
        dist_start = sqrt((x1 - circle_x)**2 + (y1 - circle_y)**2)
        dist_end = sqrt((x2 - circle_x)**2 + (y2 - circle_y)**2)
        if dist_start <= radius + self.margin or dist_end <= radius + self.margin: return True
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq == 0: return dist_start <= radius + self.margin
        t = ((circle_x - x1) * dx + (circle_y - y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        dist_closest = sqrt((closest_x - circle_x) ** 2 + (closest_y - circle_y) ** 2)
        return dist_closest <= radius + self.margin
    
    def make_detecting_vector(self, psi, for_visual=False):
        detecting_points = np.zeros([self.angle_num + 1, 3])
        psi_deg = degrees(psi)
        angle_list = np.linspace(psi_deg - self.yaw_range / 2, psi_deg + self.yaw_range / 2, self.angle_num + 1)
        for j, angle in enumerate(angle_list):
            normalized_angle = ((angle + 180) % 360) - 180
            detecting_points[j][0] = cos(radians(normalized_angle))
            detecting_points[j][1] = -sin(radians(normalized_angle)) if for_visual else sin(radians(normalized_angle))
            detecting_points[j][2] = normalized_angle
        return detecting_points

    def delete_vector_inside_obstacle(self, detecting_points, psi_ned, x_ned, y_ned):
        static_OB_data = []
        for obstacle in self.obstacles:
            if hasattr(obstacle, 'first_point') and hasattr(obstacle, 'last_point'):
                psi_deg = degrees(psi_ned)
                cr = cos(radians(psi_deg))
                sr = sin(radians(psi_deg))
                begin_x = x_ned + (-obstacle.first_point.x) * cr - obstacle.first_point.y * sr
                begin_y = y_ned + (-obstacle.first_point.x) * sr + obstacle.first_point.y * cr
                end_x = x_ned + (-obstacle.last_point.x) * cr - obstacle.last_point.y * sr
                end_y = y_ned + (-obstacle.last_point.x) * sr + obstacle.last_point.y * cr
                static_OB_data.append(('segment', begin_x, begin_y, end_x, end_y))
            elif hasattr(obstacle, 'center') and hasattr(obstacle, 'radius'):
                psi_deg = degrees(psi_ned)
                cr = cos(radians(psi_deg))
                sr = sin(radians(psi_deg))
                center_x = x_ned + (-obstacle.center.x) * cr - obstacle.center.y * sr
                center_y = y_ned + (-obstacle.center.x) * sr + obstacle.center.y * cr
                static_OB_data.append(('circle', center_x, center_y, obstacle.radius))
        non_cross_vector = []
        for i in range(self.angle_num + 1):
            cross_detected = False
            start_x, start_y = x_ned, y_ned
            end_x = x_ned + detecting_points[i][0] * self.range
            end_y = y_ned + detecting_points[i][1] * self.range
            for ob in static_OB_data:
                if ob[0] == 'segment':
                    _, seg_x1, seg_y1, seg_x2, seg_y2 = ob
                    if self.cross_check_segment(start_x, start_y, end_x, end_y, seg_x1, seg_y1, seg_x2, seg_y2):
                        cross_detected = True; break
                elif ob[0] == 'circle':
                    _, circle_x, circle_y, radius = ob
                    if self.cross_check_circle(start_x, start_y, end_x, end_y, circle_x, circle_y, radius):
                        cross_detected = True; break
            if not cross_detected:
                angle = detecting_points[i][2]
                non_cross_vector.append(angle)
        if not non_cross_vector:
            rospy.logwarn(colored("❗ 모든 벡터가 교차합니다.", 'red'))
        self.non_cross_vector_len = len(non_cross_vector)
        return non_cross_vector

    def vector_choose(self, non_cross_vector):
        if len(self.WP_k) < 2 and self.buoy_target_heading is None:
            rospy.logwarn(colored("웨이포인트 또는 부표 목표가 부족합니다.", 'red'))
            return degrees(self.psi_ned)
            
        target_angle_deg = degrees(self.target_heading)
        min_diff = float('inf')
        vector_desired = 0
        for vec in non_cross_vector:
            diff = abs(vec - target_angle_deg)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                vector_desired = vec
        return vector_desired

    def thrust_pid_controller(self, psi_ned, x_ned, y_ned):
    # 장애물 회피 후 안전 벡터 선택
        non_cross_vector = self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned)
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)

        # 목표 회전 각도 계산
        control_angle = (psi_desire - degrees(psi_ned) + 180) % 360 - 180

        # 장애물이나 벡터 차단 시 처리
        if getattr(self, 'vector_blocked', False):  # 벡터 차단 여부
            rospy.logwarn("Backward")
            self.thruster_p = 1400  # 기본 추력
            self.thruster_s = 1400  # 기본 추력
            return self.thruster_p, self.thruster_s

        # 회전 각도가 10도 이상일 때
        if abs(control_angle) >= 10:
            Re_diff = 150  # 추력 차이 (동적으로 조정 가능)
            # 우회전
            if 180 > control_angle >= 0:
                self.thruster_p = 1500 + Re_diff  # 아두이노 코드 상에서 우측 추진력
                self.thruster_s = 1500 - Re_diff  # 아두이노 코드 상에서 좌측 추진력
            # 좌회전
            elif -180 <= control_angle < 0:
                self.thruster_p = 1500 - Re_diff  # 아두이노 코드 상에서 좌측 추진력
                self.thruster_s = 1500 + Re_diff  # 아두이노 코드 상에서 우측 추진력

        # 회전 각도가 10도 이하일 때 PID 제어 적용
        elif abs(control_angle) < 10:
            # PID 제어
            cp_thrust = self.kp_thruster * control_angle
            yaw_rate = self.r_ned  # yaw rate (회전 속도)
            cd_thrust = self.kd_thruster * (-yaw_rate)

            thrust_diff = cp_thrust + cd_thrust  # 총 추력 차이

            base_thrust = self.base_thrust
            left_thrust = base_thrust + thrust_diff  # 좌측 추진력
            right_thrust = base_thrust - thrust_diff  # 우측 추진력

            # 추력 범위에 맞게 제한
            self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
            self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    def publish_tf(self, event=None):
        current_time = rospy.Time.now()
        map_transform = TransformStamped()
        map_transform.header.stamp = current_time
        map_transform.header.frame_id = "world"
        map_transform.child_frame_id = "map"
        map_transform.transform.translation.x, map_transform.transform.translation.y, map_transform.transform.translation.z = 0.0, 0.0, 0.0
        map_transform.transform.rotation.x, map_transform.transform.rotation.y, map_transform.transform.rotation.z, map_transform.transform.rotation.w = 0.0, 0.0, 0.0, 1.0
        base_link_transform = TransformStamped()
        base_link_transform.header.stamp = current_time
        base_link_transform.header.frame_id = "map"
        base_link_transform.child_frame_id = "base_link"
        base_link_transform.transform.translation.x, base_link_transform.transform.translation.y, base_link_transform.transform.translation.z = self.x_ned, self.y_ned, 0.0
        quaternion = quaternion_from_euler(0, 0, self.psi_ned)
        base_link_transform.transform.rotation.x, base_link_transform.transform.rotation.y, base_link_transform.transform.rotation.z, base_link_transform.transform.rotation.w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        self.tf_broadcaster.sendTransform([map_transform, base_link_transform])

    def publish_scanner_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "scanner"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 0.0, 0.0, 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q[0], q[1], q[2], q[3]
        self.tf_broadcaster.sendTransform(t)

    def visualize(self, visual_detecting_points, non_cross_vector, vector_desired, x_ned, y_ned):
        marker_array = MarkerArray()
        # GPS marker
        gps_marker = Marker()
        gps_marker.header.frame_id = "map"
        gps_marker.header.stamp = rospy.Time.now()
        gps_marker.ns = "gps_position"
        gps_marker.id = 0
        gps_marker.type = Marker.SPHERE
        gps_marker.action = Marker.ADD
        gps_marker.pose.position.x, gps_marker.pose.position.y, gps_marker.pose.position.z = x_ned, y_ned, 0
        gps_marker.scale.x, gps_marker.scale.y, gps_marker.scale.z = 0.5, 0.5, 0.5
        gps_marker.color.r, gps_marker.color.g, gps_marker.color.b, gps_marker.color.a = 0.0, 0.0, 1.0, 1.0
        marker_array.markers.append(gps_marker)

        # Waypoint markers
        for idx, wp in enumerate(self.wp_manager.WP_data):
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = rospy.Time.now()
            wp_marker.ns = "waypoints"
            wp_marker.id = idx + 100
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x, wp_marker.pose.position.y, wp_marker.pose.position.z = wp.x.data, wp.y.data, 0
            wp_marker.scale.x, wp_marker.scale.y, wp_marker.scale.z = 0.3, 0.3, 0.3
            if self.WP_k and wp.num.data == self.WP_k[1].num.data and self.ch:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 1.0, 0.0, 1.0
            else:
                wp_marker.color.r, wp_marker.color.g, wp_marker.color.b, wp_marker.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(wp_marker)
            range_marker = Marker()
            range_marker.header.frame_id = "map"
            range_marker.header.stamp = rospy.Time.now()
            range_marker.ns = "waypoint_ranges"
            range_marker.id = idx + 1000
            range_marker.type = Marker.CYLINDER
            range_marker.action = Marker.ADD
            range_marker.pose.position.x, range_marker.pose.position.y, range_marker.pose.position.z = wp.x.data, wp.y.data, 0.0
            range_marker.scale.x, range_marker.scale.y, range_marker.scale.z = wp.range.data * 2, wp.range.data * 2, 0.01
            range_marker.color.r, range_marker.color.g, range_marker.color.b, range_marker.color.a = 0.5, 0.0, 0.0, 0.3
            marker_array.markers.append(range_marker)

        # Vector visualization
        for idx, point in enumerate(visual_detecting_points):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "vectors"
            arrow_marker.id = idx + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x, arrow_marker.scale.y, arrow_marker.scale.z = 0.1, 0.05, 0.05
            start_point = Point(x_ned, y_ned, 0)
            end_point = Point(x_ned + point[0] * self.range, y_ned + point[1] * self.range, 0)
            arrow_marker.points = [start_point, end_point]
            if point[2] == vector_desired:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = 1.0, 0.0, 0.0, 1.0
            elif point[2] in non_cross_vector:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = 0.0, 0.0, 1.0, 0.8
            else:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = 0.5, 0.5, 0.5, 0.4
            marker_array.markers.append(arrow_marker)
        self.marker_array_pub.publish(marker_array)

    def print_state(self):
        separator = "=" * 50
        print(colored(separator, "cyan"))
        print(colored("🚀 현재 보트 상태", "yellow", attrs=["bold"]))
        print(colored(separator, "cyan"))
        print(f"🧭 {colored('현재 헤딩[psi_ned]', 'blue')}: {degrees(self.psi_ned):.2f}°")
        print(f"🎯 {colored('목표 헤딩[target_heading]', 'blue')}: {degrees(self.target_heading):.2f}°")
        print(f"📏 {colored('목표까지 거리[d_goal]', 'magenta')}: {self.d_goal:.3f} m")
        print(f"  🚀 {colored('추진 출력', 'red')}: 좌측 = {self.thruster_p}, 우측 = {self.thruster_s}")
        print(f"🛡️ {colored('도달 가능한 벡터 수', 'cyan')}: {self.non_cross_vector_len}")
        print(f"🧭 {colored('제어각도[control_angle]', 'yellow')}: {self.control_angle:.4f}°")
        print(f"🧭 {colored('최적벡터[psi_desire]', 'blue')}: {self.psi_desire:.2f}°")
        direction = "◀ 좌회전" if self.control_angle < 0 else "▶ 우회전"
        print(colored(f"✅ 선택된 벡터: {self.vector_desired:.2f}° {direction}", "green"))
        print(colored(separator, "cyan"))
        
def main():
    ship = SHIP()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ship.pose_received:
            ship.publish_tf()
            ship.publish_scanner_tf()
            ship.ship_run()
            ship.print_state()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass