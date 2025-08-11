#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import UInt16
from math import sin, cos, radians, degrees, hypot, atan2, pi, sqrt, isnan, isinf
import numpy as np
from tricat_msgs.msg import Pose, Control
from obstacle_detector.msg import Obstacles
from ship.wp_manager import WpManager
from control.autopilot import heading_cal
import math

##### 시각화를 위한 임포트 #####
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

##### 터미널 색깔 추가 ######
from termcolor import colored

##### 카메라 및 도킹 관련 임포트 #####
import cv2
import time
from docking.image_sensing import *

D2R = pi / 180
R2D = 180 / pi

class SHIP:
    def __init__(self):
        ### GPS(Waypoint) ###
        self.wp_manager=WpManager()
        self.wp_manager.wp_client()
        self.WP_data = []
        self.WP_k = []
        self.num_k = 0
        self.d_goal = 0
        self.target_heading = 0.0  # target_heading 기본값 추가
        self.psi_diff= 0.0    # (추가) target_angle도 함께 초기화
        self.ch = False
        
        if self.wp_manager.WP_data:
            self.wp_manager.initialize()

        ### /Pose ###
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.u_ned = 0.0
        self.v_ned = 0.0
        self.r_ned = 0.0
        self.U = 0.0

        ### psi_d ### 
        self.psi_d_ned = 0.0

        # sensor_total.py 실행시키고 navigation.py실행시켜서 Pose 토픽 받음
        self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)

        ### LiDAR ###
        rospy.Subscriber("/obstacles", Obstacles, self.obstacle_callback, queue_size=10)
        self.range = rospy.get_param("range", 10)
        self.angle_num = rospy.get_param("angle_num", 10)
        self.yaw_range = rospy.get_param("yaw_range", 60)
        self.margin = rospy.get_param("margin", 0.5)
        self.goal_range = rospy.get_param("goal_range", 2)
        ### 장애물 과 벡터 ###
        self.obstacles = [] 
        self.vector_begin = np.zeros((self.angle_num + 1, 2))  # 초기화
        self.vector_end = np.zeros((self.angle_num + 1, 2)) 
        self.non_cross_vector_len = 0
        self.vector_blocked = False
        self.last_vector_desired = None

        ### Control ### (추력제어 코드 나오면 추력으로 바꾸기, 현재는 서보로 일단 ㄱ)
        self.control_msg = Control()
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.thruster_s = 0
        self.thruster_p = 0
        self.base_thrust = rospy.get_param("base_thrust", 1500)  # base_thrust 추가
        self.thrust_range = rospy.get_param("thrust_range", [1100, 1900] )
        # heading autopilot
        self.kp_thruster = rospy.get_param("kp_thruster", 1.7)
        self.kd_thruster = rospy.get_param("kd_thruster" , 0.3)
        
        # visual
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        self.arrival_range = rospy.get_param("goal_range", 10)
        
        ##### 카메라 및 도킹 관련 초기화 #####
        self.control_mode = 'Avoidance'  # Avoidance -> Hopping -> Docking
        self.camera_initialized = False
        self.cap = None
        self.sensing = None
        
        # 도킹 관련 변수
        self.docking_wp_num = 3  # 도킹 시작 웨이포인트 번호
        self.hopping_wp_num = 2   # 호핑 시작 웨이포인트 번호
        self.stop_area_threshold = rospy.get_param("stop_area_threshold", 30000)
        self.aline = False
        self.tracking = True
        
        
    def pose_callback(self, msg):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)
            psi_deg = float(msg.psi.data)
            self.psi_ned = ((psi_deg * D2R + pi) % (2 * pi)) - pi  # deg -> rad

            rospy.loginfo_throttle(2, f"Pose 수신: x={self.x_ned}, y={self.y_ned}, psi(deg)={psi_deg}, psi(rad)={self.psi_ned}")

            self.pose_received = True
            self.publish_tf(None)  # 수신 시 즉시 tf 퍼블리시

        except Exception as e:
            rospy.logwarn(f"Pose 값 수신 오류: {e}")


    def obstacle_callback(self, msg):
        self.obstacles = msg.circles + msg.segments
       
        
    def initialize_camera(self):
        """카메라 초기화"""
        if self.camera_initialized:
            return True
            
        try:
            video_num = rospy.get_param('video_num', 1)
            self.cap = cv2.VideoCapture(0)  
            if not self.cap.isOpened():
                rospy.logerr("웹캠을 열 수 없습니다.")
                return False

            # 카메라 해상도 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)  
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2560) 

            target_shape = rospy.get_param("target_shape", 'Cross')
            color_to_detect = rospy.get_param("color_to_detect", 'blue')
            self.sensing = Sensing(target_shape, color_to_detect)
            
            self.camera_initialized = True
            rospy.loginfo("카메라 초기화 완료")
            return True
            
        except Exception as e:
            rospy.logerr(f"카메라 초기화 실패: {e}")
            return False
            
       
    def ship_run(self):
        """장애물 회피 모드 실행"""
        ### ship ###
        self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)
        self.target_angle = self.target_heading - self.psi_ned

        ### Control ###
        self.thruster_p, self.thruster_s = self.Avoidance_control(self.psi_ned, self.x_ned, self.y_ned)
        ### Vector ###
        self.detecting_points = self.make_detecting_vector(self.psi_ned)
        self.non_cross_vector = self.delete_vector_inside_obstacle(self.detecting_points, self.psi_ned, self.x_ned, self.y_ned)
        self.vector_choose(self.non_cross_vector, self.x_ned, self.y_ned)
        self.vector_desired = self.vector_choose(self.non_cross_vector, self.x_ned, self.y_ned)
        ### Visual ###
        # self.visual_detecting_points = self.make_detecting_vector(self.psi_ned, for_visual=True)  # 시각화용은 y반전
        # self.visualize(self.visual_detecting_points, self.non_cross_vector, self.vector_desired, self.x_ned, self.y_ned)
    

    def cross_check_segment(self, x1, y1, x2, y2, seg_x1, seg_y1, seg_x2, seg_y2):
        """선분 교차 여부 검사"""
        denominator = (seg_x2 - seg_x1) * (y2 - y1) - (seg_y2 - seg_y1) * (x2 - x1)
        if abs(denominator) < 1e-10:
            return False  # 평행하거나 겹침

        ua = ((seg_y2 - seg_y1) * (x1 - seg_x1) - (seg_x2 - seg_x1) * (y1 - seg_y1)) / denominator
        ub = ((y2 - y1) * (x1 - seg_x1) - (x2 - x1) * (y1 - seg_y1)) / denominator

        if 0 <= ua <= 1 and 0 <= ub <= 1:
            return True  # 교차함
        return False
    
    def cross_check_circle(self, x1, y1, x2, y2, circle_x, circle_y, radius):
        """ 
        선분 (x1, y1)-(x2, y2)와 원 (circle_x, circle_y, radius) 교차 여부 확인 
        1. 시작/끝점이 원 내부에 있으면 무조건 교차
        2. 선분과 원의 최단 거리 ≤ radius + margin이면 교차
        """

        # 1. 벡터 시작/끝점이 원 내부에 있는지 확인
        dist_start = sqrt((x1 - circle_x)**2 + (y1 - circle_y)**2)
        dist_end = sqrt((x2 - circle_x)**2 + (y2 - circle_y)**2)
        if dist_start <= radius + self.margin or dist_end <= radius + self.margin:
            return True  # 시작 또는 끝점이 원 내부 → 교차

        # 2. 선분과 원 중심 간 최단 거리 계산
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy

        # 선분 길이가 0인 경우 (벡터 길이가 없음)
        if length_sq == 0:
            return dist_start <= radius + self.margin

        # 투영 파라미터 t (선분상의 최근접 점 위치)
        t = ((circle_x - x1) * dx + (circle_y - y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))  # 선분 범위 [0, 1]로 제한

        # 선분상의 최근접 점 좌표
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # 최근접 점과 원 중심 간 거리
        dist_closest = sqrt((closest_x - circle_x) ** 2 + (closest_y - circle_y) ** 2)

        return dist_closest <= radius + self.margin  # 교차 여부 반환
    
    def make_detecting_vector(self, psi, for_visual=False):
        detecting_points = np.zeros([self.angle_num + 1, 3])
        angle_list = np.linspace(psi - self.yaw_range / 2, psi + self.yaw_range / 2, self.angle_num + 1)

        for j, angle in enumerate(angle_list):
            normalized_angle = ((angle + 180) % 360) - 180
            detecting_points[j][0] = cos(radians(normalized_angle))
            detecting_points[j][1] = -sin(radians(normalized_angle)) if for_visual else sin(radians(normalized_angle))
            detecting_points[j][2] = normalized_angle

        return detecting_points


    def delete_vector_inside_obstacle(self, detecting_points, psi_ned, x_ned, y_ned):
        static_OB_data = []

        # 장애물 데이터 정리
        for obstacle in self.obstacles:
            if hasattr(obstacle, 'first_point') and hasattr(obstacle, 'last_point'):
                begin_x = x_ned + (-obstacle.first_point.x) * cos(radians(psi_ned)) - obstacle.first_point.y * sin(radians(psi_ned))
                begin_y = y_ned + (-obstacle.first_point.x) * sin(radians(psi_ned)) + obstacle.first_point.y * cos(radians(psi_ned))
                end_x = x_ned + (-obstacle.last_point.x) * cos(radians(psi_ned)) - obstacle.last_point.y * sin(radians(psi_ned))
                end_y = y_ned + (-obstacle.last_point.x) * sin(radians(psi_ned)) + obstacle.last_point.y * cos(radians(psi_ned))
                static_OB_data.append(('segment', begin_x, begin_y, end_x, end_y))

            elif hasattr(obstacle, 'center') and hasattr(obstacle, 'radius'):
                center_x = x_ned + (-obstacle.center.x) * cos(radians(psi_ned)) - obstacle.center.y * sin(radians(psi_ned))
                center_y = y_ned + (-obstacle.center.x) * sin(radians(psi_ned)) + obstacle.center.y * cos(radians(psi_ned))
                static_OB_data.append(('circle', center_x, center_y, obstacle.radius))

        non_cross_vector = []

        # 각 탐지 벡터와 장애물 교차 여부 확인
        for i in range(self.angle_num + 1):
            cross_detected = False
            start_x, start_y = x_ned, y_ned
            end_x = x_ned + detecting_points[i][0] * self.range
            end_y = y_ned + detecting_points[i][1] * self.range

            for ob in static_OB_data:
                if ob[0] == 'segment':
                    _, seg_x1, seg_y1, seg_x2, seg_y2 = ob
                    if self.cross_check_segment(start_x, start_y, end_x, end_y, seg_x1, seg_y1, seg_x2, seg_y2):
                        cross_detected = True
                        break

                elif ob[0] == 'circle':
                    _, circle_x, circle_y, radius = ob
                    if self.cross_check_circle(start_x, start_y, end_x, end_y, circle_x, circle_y, radius):
                        cross_detected = True
                        break

            # 교차하지 않는 벡터만 추가
            if not cross_detected:
                angle = detecting_points[i][2]
                non_cross_vector.append(angle)

        # 모든 벡터가 교차하면 중앙 각도 강제 추가
        if not non_cross_vector:
            rospy.logwarn("모든 벡터가 교차합니다.")
            self.vector_blocked = True  # 상태 저장
        else:
            self.vector_blocked = False  # 정상

        self.non_cross_vector_len = len(non_cross_vector)
        return non_cross_vector

    
    # Step3. choose vector
    def vector_choose(self, non_cross_vector, x_ned, y_ned):
        if len(self.WP_k) < 2:
            rospy.logwarn("웨이포인트가 부족합니다.")
            return 0

        # 목표 방향 (deg)
        target_angle = degrees(self.target_heading)  # 이미 ship_run에서 계산됨
        self.target_angle = (target_angle - degrees(self.psi_ned) + 180) % 360 - 180  # 보트 기준 회전각

        min_diff = float('inf')
        vector_desired = 0

        for vec in non_cross_vector:
            diff = abs(vec - target_angle)
            if diff > 180:
                diff = 360 - diff  # 각도 차이 최소화

            if diff < min_diff:
                min_diff = diff
                vector_desired = vec

        return vector_desired

    # Step4. Thrust-based PID control
    def Avoidance_control_1(self, psi_ned, x_ned, y_ned):
        non_cross_vector = self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned)

        if getattr(self, 'vector_blocked', False):  # vector_block
            rospy.logwarn("Backward")

            self.control_angle = degrees(self.target_angle)
            self.psi_desire = self.target_heading
            B_diff = 150
            if -180 < self.control_angle <= 0:
                self.thruster_p = 1600 - B_diff 
                self.thruster_s = 1600 + B_diff
            elif 0 < self.control_angle < 180:
                self.thruster_p = 1600 + B_diff
                self.thruster_s = 1600 - B_diff
            return self.thruster_p, self.thruster_s

        # 벡터 선택 및 각도 계산
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)
        control_angle = (psi_desire - degrees(psi_ned) + 180) % 360 - 180

        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180

        self.control_angle = control_angle 
        self.psi_desire = psi_desire

        # PID 제어기
        cp_thrust = self.kp_thruster * control_angle
        yaw_rate = self.r_ned
        cd_thrust = self.kd_thruster * (-yaw_rate)

        thrust_diff = cp_thrust + cd_thrust
        base_thrust = self.base_thrust
        left_thrust = 3000 - (base_thrust + thrust_diff)
        right_thrust = 3000 - (base_thrust - thrust_diff)

        self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s
    
    def Avoidance_control_2(self, psi_ned, x_ned, y_ned):
        non_cross_vector = self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned)
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)
        if getattr(self, 'vector_blocked', False):  # vector_block
            rospy.logwarn("Backward")

            self.control_angle = degrees(self.target_angle)
            self.psi_desire = self.target_heading
            B_diff = 150
            if -180 < self.control_angle <= 0:
                self.thruster_p = 1600 - B_diff
                self.thruster_s = 1600 + B_diff
            elif 0 < self.control_angle < 180:
                self.thruster_p = 1600 + B_diff
                self.thruster_s = 1600 - B_diff
            return self.thruster_p, self.thruster_s
        control_angle = (psi_desire - degrees(psi_ned) + 180) % 360 - 180

        if abs(control_angle) > self.yaw_range/5:
            Re_diff = 150
            if 180 >control_angle >= 0:
                self.thruster_p = 1500 - Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 + Re_diff
            elif -180 < control_angle < 0:
                self.thruster_p = 1500 - Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 + Re_diff

        elif abs(control_angle) <= self.yaw_range/5:
            cp_thrust = self.kp_thruster * control_angle
            yaw_rate = self.r_ned
            cd_thrust = self.kd_thruster * (-yaw_rate)

            thrust_diff = cp_thrust + cd_thrust

            base_thrust = self.base_thrust
            left_thrust = 3000-(base_thrust + thrust_diff)
            right_thrust =3000-(base_thrust - thrust_diff)

            self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
            self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s
    
    def Avoidance_control_3(self, psi_ned, x_ned, y_ned):
        # 1) 장애물 벡터/목표방위
        non_cross_vector = self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned)
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)

        # 2) 벡터 블록: 후진 + 강제 회전
        if getattr(self, 'vector_blocked', False):
            rospy.logwarn("Backward")
            self.control_angle = math.degrees(self.target_angle)
            self.psi_desire = self.target_heading
            B_diff = 150
            if -180 < self.control_angle <= 0:
                self.thruster_p = 1600 - B_diff
                self.thruster_s = 1600 + B_diff
            elif 0 < self.control_angle < 180:
                self.thruster_p = 1600 + B_diff
                self.thruster_s = 1600 - B_diff
            return self.thruster_p, self.thruster_s

        # 3) 각도 오차
        control_angle = (psi_desire - math.degrees(psi_ned) + 180) % 360 - 180
        self.control_angle = control_angle
        self.psi_desire = psi_desire
        abs_e = abs(control_angle)

        # 4) 드리프트용 추력차 맵핑
        pivot_deg = 50.0
        k_below   = 6.0
        k_above   = 8.0
        deadband_deg = 0.0

        if abs_e <= deadband_deg:
            thrust_diff = 0.0
        elif abs_e <= pivot_deg:
            thrust_diff = k_below * abs_e
        else:
            thrust_diff = 300.0 + k_above * (abs_e - pivot_deg)

        diff_max = getattr(self, "diff_max_drift", 700)
        thrust_diff = max(min(thrust_diff, diff_max), 0.0)

        base = self.base_thrust

        if control_angle > 0:
            # 목표가 좌측(반시계) → 좌 후진 / 우 전진
            self.outinfo = f"↗ 우측 드리프트 | diff={int(thrust_diff)}"
            self.thruster_p = base - int(thrust_diff)
            self.thruster_s = base + int(thrust_diff)
        else:
            # 목표가 우측(시계) → 좌 전진 / 우 후진
            self.outinfo = f"↖ 좌측 드리프트 | diff={int(thrust_diff)}"
            self.thruster_p = base + int(thrust_diff)
            self.thruster_s = base - int(thrust_diff)

            self.thruster_p = max(min(self.thruster_p, self.thrust_range[1]), self.thrust_range[0])
            self.thruster_s = max(min(self.thruster_s, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s

    def Avoidance_control_4(self, psi_ned, x_ned, y_ned):
        # 장애물 벡터 정리
        non_cross_vector = self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned)

        # 벡터 블록: 후진/강제 회전
        if getattr(self, 'vector_blocked', False):
            rospy.logwarn("Backward")

            self.control_angle = math.degrees(self.target_angle)
            self.psi_desire    = self.target_heading
            B_diff = 150
            if -180 < self.control_angle <= 0:
                self.thruster_p = 1600 - B_diff
                self.thruster_s = 1600 + B_diff
            elif 0 < self.control_angle < 180:
                self.thruster_p = 1600 + B_diff
                self.thruster_s = 1600 - B_diff
            return self.thruster_p, self.thruster_s

        # 벡터 선택 및 각도 계산
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)
        control_angle = (psi_desire - math.degrees(psi_ned) + 180) % 360 - 180

        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180

        self.control_angle = control_angle
        self.psi_desire    = psi_desire

        # P 스케줄링
        abs_e   = min(abs(control_angle), 90.0)
        k_below = 6.0
        k_above = 8.0

        if abs_e <= 50.0:
            p_mag = k_below * abs_e
        else:
            p_mag = 300.0 + k_above * (abs_e - 50.0)

        cp_thrust = p_mag if control_angle >= 0 else -p_mag

        yaw_rate  = self.r_ned
        cd_thrust = self.kd_thruster * (-yaw_rate)

        thrust_diff = cp_thrust + cd_thrust

        base_thrust  = self.base_thrust
        left_thrust  = 3000-(base_thrust + thrust_diff)
        right_thrust = 3000-(base_thrust - thrust_diff)

        self.thruster_p = max(min(left_thrust,  self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s

    def Avoidance_control(self, psi_ned, x_ned, y_ned):
        non_cross_vector = self.delete_vector_inside_obstacle(
            self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned
        )

        if getattr(self, 'vector_blocked', False):  # vector_block
            rospy.logwarn("Backward")

            self.control_angle = degrees(self.target_angle)
            self.psi_desire = self.target_heading
            B_diff = 150
            if -180 < self.control_angle <= 0:
                self.thruster_p = 1600 - B_diff
                self.thruster_s = 1600 + B_diff
            elif 0 < self.control_angle < 180:
                self.thruster_p = 1600 + B_diff
                self.thruster_s = 1600 - B_diff
            return self.thruster_p, self.thruster_s

        # === 1) 목표 방위 계산 & 오차 정규화 ===
        psi_desire = self.vector_choose(non_cross_vector, x_ned, y_ned)
        control_angle = (psi_desire - degrees(psi_ned) + 180) % 360 - 180
        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180

        self.control_angle = control_angle
        self.psi_desire = psi_desire

        cp_thrust = self.kp_thruster * control_angle
        yaw_rate = self.r_ned
        cd_thrust = self.kd_thruster * (-yaw_rate)

        thrust_diff = cp_thrust + cd_thrust

        # 좌우 추력차 최대값 제한
        thrust_diff = max(min(thrust_diff, 150.0), -150.0)

        # === 4) 좌우 추력 산출 ===
        base_thrust = self.base_thrust
        left_thrust  = 3000 - (base_thrust + thrust_diff)
        right_thrust = 3000 - (base_thrust - thrust_diff)

        self.thruster_p = max(min(left_thrust,  self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s


    def Hopping_control(self, psi_ned):
        psi_desire = self.target_heading
        control_angle = (psi_desire - psi_ned + pi) % (2 * pi) - pi
        control_angle_deg = degrees(control_angle)
        control_angle_deg = ((control_angle_deg) + 180) % 360 - 180
        self.psi_desire = psi_desire
        self.control_angle_deg = control_angle_deg

        if abs(control_angle_deg) > self.yaw_range/10:
            Re_diff = 150
            if 180 >control_angle_deg >= 0:
                self.thruster_p = 1500 - Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 + Re_diff
            elif -180 < control_angle_deg < 0:
                self.thruster_p = 1500 + Re_diff # 아두이노 코드 ㅄ
                self.thruster_s = 1500 - Re_diff
        else:
            cp_thrust = self.kp_thruster * control_angle_deg
            yaw_rate = self.r_ned
            cd_thrust = self.kd_thruster * (-yaw_rate)

            thrust_diff = cp_thrust + cd_thrust

            base_thrust = self.base_thrust
            left_thrust = 3000 - (base_thrust + thrust_diff)
            right_thrust =3000 - (base_thrust - thrust_diff)

            self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
            self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s
    
        #######################################################Docking###############################################################
    def docking_process(self):
        """도킹 프로세스 실행"""
        if not self.camera_initialized:
            if not self.initialize_camera():
                return False
                
        ret, frame = self.cap.read()
        if not ret:
            rospy.logerr("프레임을 가져올 수 없습니다.")
            return False

        # 프레임의 윗부분 1/4 잘라내기
        height = frame.shape[0]
        cropped_frame = frame[height // 4:, :]  # 윗부분 1/4 잘라내기

        # 도형 인식 시도
        detected, shape_img, largest_shape = self.sensing.process_once(cropped_frame)
        cv2.imshow('Raw Image', cropped_frame)  # 잘라낸 원본 이미지 표시
        cv2.imshow('Detected Shapes', shape_img)  # 검출된 도형 이미지 표시
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False

        if not detected and not self.aline:  # 인식이 안되면 계속 왼쪽으로 회전
            rospy.loginfo("왼 회전 중")
            self.thruster_p = 1480
            self.thruster_s = 1520
            return True

        if detected:
            area, contour, _ = largest_shape
            # contour_points 함수 사용
            if contour is not None and len(contour) > 0:
                _, shape_center = contour_points(contour)
            else:
                shape_center = None

            if area < self.stop_area_threshold:
                if shape_center and cropped_frame is not None:
                    center_x = shape_center[1]  # 도형의 중심선
                    line_position = cropped_frame.shape[1] // 2  # 카메라 중앙선
                    error_x = center_x - line_position

                    k_p = 0.03  # 고정값

                    self.thruster_p = int(np.clip(1500 - k_p * error_x, 1350, 1650))
                    self.thruster_s = int(np.clip(1500 - k_p * error_x, 1350, 1650))
            else:
                if not self.aline:
                    rospy.loginfo("정렬완료. 앞으로 갈 준비 완료.")
                    rospy.sleep(1)  
                    start_time = time.time()
                    duration = 1
                    self.thruster_p = 1500
                    self.thruster_s = 1500
                    rospy.loginfo("전진중...")

                    while time.time() - start_time < duration:
                        self.control_publish()
                        rospy.sleep(0.1)

                    rospy.loginfo("전진완료.")
                    self.aline = True
                    return False
        return True
        
        
    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)
    
################################################## 시각화 ############################################################3

############## 터미널 출력에 신경을 많이 쓰자자 ####################
    def print_state(self):
        separator = "=" * 50
        print(colored(separator, "cyan"))
        print(colored("현재 보트 상태", "yellow", attrs=["bold"]))
        print(colored(separator, "cyan"))

        # 제어 모드 표시
        mode_colors = {
            'Avoidance': 'green',
            'Hopping': 'yellow', 
            'Docking': 'red'
        }
        mode_color = mode_colors.get(self.control_mode, 'white')
        print(f"{colored('제어 모드', mode_color, attrs=['bold'])}: {colored(self.control_mode, mode_color)}")

        # 웨이포인트 및 위치 정보
        print(f"{colored('웨이포인트 번호', 'green')}: {self.WP_k[1].num.data}")
        print(f"{colored('목표 위치', 'green')}: x = {self.WP_k[1].x.data:.3f}, y = {self.WP_k[1].y.data:.3f}")
        print(f"{colored('현재 위치', 'green')}: x = {self.x_ned:.3f}, y = {self.y_ned:.3f}")

        # 헤딩 및 거리 정보
        print(f"{colored('현재 헤딩[psi_ned]', 'blue')}: {degrees(self.psi_ned):.2f}°")
        print(f"{colored('목표 헤딩[target_heading]', 'blue')}: {degrees(self.target_heading):.2f}°")
        print(f"{colored('목표 각도[psi_diff]', 'yellow')}: {self.target_angle:.4f}°")
        print(f"{colored('목표까지 거리[d_goal]', 'magenta')}: {self.d_goal:.3f} m")

        # 제어 상태
        print(colored("제어 상태:", "yellow", attrs=["bold"]))
        print(f"  {colored('추진 출력', 'red')}: 좌측 = {int(self.thruster_p)}, 우측 = {int(self.thruster_s)}")

        # 모드별 추가 정보
        if self.control_mode == 'Avoidance':
            if getattr(self, 'vector_blocked', False):
                print(colored("모든 벡터가 막혀 있습니다! 역추진 중입니다.", "red", attrs=["bold"]))
            else:
                print(f"{colored('도달 가능한 벡터 수', 'cyan')}: {getattr(self, 'non_cross_vector_len', 0)}")
                if hasattr(self, 'control_angle'):
                    print(f"{colored('제어각도[control_angle]', 'yellow')}: {self.control_angle:.4f}°")
                if hasattr(self, 'psi_desire'):
                    print(f"{colored('최적벡터[psi_desire]', 'blue')}: {self.psi_desire:.2f}°")
                if hasattr(self, 'vector_desired') and hasattr(self, 'control_angle'):
                    direction = "좌회전" if self.control_angle < 0 else "우회전"
                    print(colored(f"선택된 벡터: {self.vector_desired:.2f}° {direction}", "green"))
                
        elif self.control_mode == 'Hopping':
            if hasattr(self, 'control_angle_deg'):
                print(f"{colored('호핑 제어각도', 'yellow')}: {self.control_angle_deg:.2f}°")
            else:
                print(f"{colored('호핑 제어각도', 'yellow')}: 계산 중...")
            
        elif self.control_mode == 'Docking':
            print(f"{colored('카메라 상태', 'blue')}: {'초기화됨' if self.camera_initialized else '초기화 안됨'}")
            print(f"{colored('정렬 상태', 'yellow')}: {'완료' if self.aline else '진행중'}")

        print(colored(separator, "cyan"))

        
def main():
    rospy.init_node("ship", anonymous=True)
    rate = rospy.Rate(10)
    ship = SHIP()
    
    while not rospy.is_shutdown():
        # ship.publish_tf(None)
        # ship.publish_scanner_tf()
        
        # 웨이포인트 관리
        try:
            ship.d_goal = ship.wp_manager.cal_d_goal(ship.x_ned, ship.y_ned)
            ship.WP_k = ship.wp_manager.manage(ship.x_ned, ship.y_ned)
            
            if len(ship.WP_k) < 2:
                rospy.logwarn("웨이포인트가 부족합니다.")
                rate.sleep()
                continue
        except Exception as e:
            rospy.logerr(f"웨이포인트 관리 오류: {e}")
            rate.sleep()
            continue
            
        current_wp_num = ship.WP_k[1].num.data
        
        # === 변경된 분기: 1번=회피, 2번=호핑, 3번=도킹 ===
        if current_wp_num == 3:
            # 도킹 모드
            if ship.control_mode != "Docking":
                ship.control_mode = "Docking"
                rospy.loginfo("Docking_mode")
            ship.tracking = ship.docking_process()
            if not ship.tracking:
                rospy.loginfo("도킹 완료!")
                break

        elif current_wp_num == 2:
            # 호핑 모드
            if ship.control_mode != "Hopping":
                ship.control_mode = "Hopping"
                rospy.loginfo("Hopping_mode")
            ship.Hopping_control(ship.psi_ned)

        elif current_wp_num == 1:
            # 회피 모드
            if ship.control_mode != "Avoidance":
                ship.control_mode = "Avoidance"
                rospy.loginfo("Avoidance_mode")
            ship.ship_run()

        else:
            # 정의되지 않은 번호는 기본 회피 모드로 처리(안전용)
            if ship.control_mode != "Avoidance":
                ship.control_mode = "Avoidance"
                rospy.loginfo("Avoidance_mode")
            ship.ship_run()
        
        ship.control_publish()
        ship.print_state()
        rate.sleep()
    
    # 정리
    if ship.cap is not None:
        ship.cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
