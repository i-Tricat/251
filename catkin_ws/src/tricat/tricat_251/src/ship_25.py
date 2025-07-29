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
        self.target_heading = 0.0  # ✅ target_heading 기본값 추가
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
        
        ### 장애물 과 벡터 ###
        self.obstacles = [] 
        self.vector_begin = np.zeros((self.angle_num + 1, 2))  # ✅ 초기화
        self.vector_end = np.zeros((self.angle_num + 1, 2)) 
        self.non_cross_vector_len = 0
        # self.vector_choose(self.delete_vector_inside_obstacle(self.make_detecting_vector(self.psi_ned),self.psi_ned, self.x_ned, self.y_ned), self.x_ned, self.y_ned)
        # self.servo_pid_controller(self.psi_ned, self.x_ned, self.y_ned)

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
         ### ✅ pose_received 속성 초기화 (pose 수신 여부 확인용)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
        self.publish_scanner_tf()
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        self.arrival_range = rospy.get_param("goal_range", 10)
        
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
       
    def ship_run(self):
        ### ship ###
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)
        self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)
        self.target_angle = self.target_heading - self.psi_ned

        ### Control ###
        self.thruster_p, self.thruster_s = self.thrust_pid_controller(self.psi_ned, self.x_ned, self.y_ned)  # ✅ 추가
        ### Vector ###
        self.detecting_points = self.make_detecting_vector(self.psi_ned)
        self.non_cross_vector = self.delete_vector_inside_obstacle(self.detecting_points, self.psi_ned, self.x_ned, self.y_ned)
        self.vector_choose(self.non_cross_vector, self.x_ned, self.y_ned)
        self.vector_desired = self.vector_choose(self.non_cross_vector, self.x_ned, self.y_ned)
        ### Visual ###
        self.visual_detecting_points = self.make_detecting_vector(self.psi_ned, for_visual=True)  # 시각화용은 y반전
        self.visualize(self.visual_detecting_points, self.non_cross_vector, self.vector_desired, self.x_ned, self.y_ned)
    

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

        # 1️⃣ 벡터 시작/끝점이 원 내부에 있는지 확인
        dist_start = sqrt((x1 - circle_x)**2 + (y1 - circle_y)**2)
        dist_end = sqrt((x2 - circle_x)**2 + (y2 - circle_y)**2)
        if dist_start <= radius + self.margin or dist_end <= radius + self.margin:
            return True  # 시작 또는 끝점이 원 내부 → 교차

        # 2️⃣ 선분과 원 중심 간 최단 거리 계산
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
            rospy.logwarn("❗ 모든 벡터가 교차합니다.")

        self.non_cross_vector_len = len(non_cross_vector)
        return non_cross_vector

    
    # Step3. choose vector
    def vector_choose(self, non_cross_vector, x_ned, y_ned):
        if len(self.WP_k) < 2:
            rospy.logwarn("웨이포인트가 부족합니다.")
            return 0

        # ✅ 목표 방향 (deg)
        target_angle = degrees(self.target_heading)  # 이미 ship_run에서 계산됨
        self.target_angle = (target_angle - degrees(self.psi_ned) + 180) % 360 - 180  # 보트 기준 회전각

        min_diff = float('inf')
        vector_desired = 0

        for vec in non_cross_vector:
            diff = abs(vec - target_angle)
            if diff > 180:
                diff = 360 - diff  # 각도 차이 최소화 (예: -170° vs +170°는 사실상 20° 차이)

            if diff < min_diff:
                min_diff = diff
                vector_desired = vec

        return vector_desired



    # Step4. Thrust-based PID control
    def thrust_pid_controller(self, psi_ned, x_ned, y_ned):
        psi_desire = self.vector_choose(self.delete_vector_inside_obstacle(self.make_detecting_vector(psi_ned), psi_ned, x_ned, y_ned), x_ned, y_ned)
        control_angle = (psi_desire - psi_ned + 180) % 360 - 180

        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180
        
        self.control_angle = control_angle 
        self.psi_desire = psi_desire
        cp_thrust = self.kp_thruster * control_angle
        yaw_rate = degrees(self.r_ned)
        cd_thrust = self.kd_thruster * (-yaw_rate)

        thrust_diff = cp_thrust + cd_thrust  # 좌우 추진기 차등 추력 계산

        # 기본 추력 설정
        base_thrust = self.base_thrust  # 기본 전진 추력
        left_thrust = base_thrust + thrust_diff
        right_thrust = base_thrust - thrust_diff

        # 추력 범위 제한
        self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

################################################## 시각화 ########################33
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

        # ✅ 퍼블리시
        self.tf_broadcaster.sendTransform([map_transform, base_link_transform])
        # rospy.loginfo(f"✅ TF 퍼블리시: map→base_link (x={self.x_ned}, y={self.y_ned}, yaw={self.psi_ned})")

    def publish_scanner_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"   # scanner의 부모 프레임
        t.child_frame_id = "scanner"      # 생성할 scanner 프레임 이름
        t.transform.translation.x = 0.0   # 필요 시 값 조정
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        # rospy.loginfo("✅ scanner → base_link 변환 퍼블리시 완료")


    def visualize(self, visual_detecting_points, non_cross_vector, vector_desired, x_ned, y_ned):
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


            # ✅ 벡터 시각화 (화살표 마커)
        for idx, point in enumerate(visual_detecting_points):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "vectors"
            arrow_marker.id = idx + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.1  # 화살 길이
            arrow_marker.scale.y = 0.05 # 화살 두께
            arrow_marker.scale.z = 0.05

            start_point = Point(x_ned, y_ned, 0)
            end_point = Point(
                x_ned + point[0] * self.range,
                y_ned + point[1] * self.range,
                0
            )

            arrow_marker.points = [start_point, end_point]

            # ✅ 색상 지정
            if point[2] == vector_desired:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 1.0, 0.0, 0.0  # 빨강
                arrow_marker.color.a = 1.0
            elif point[2] in non_cross_vector:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 0.0, 0.0, 1.0  # 파랑
                arrow_marker.color.a = 0.8
            else:
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = 0.5, 0.5, 0.5  # 회색
                arrow_marker.color.a = 0.4

            marker_array.markers.append(arrow_marker)


        # ✅ 퍼블리시 및 확인 로그
        self.marker_array_pub.publish(marker_array)
        # rospy.loginfo("✅ 마커 퍼블리시 완료")

############## 터미널 ####################
    def print_state(self):
        separator = "=" * 50
        print(colored(separator, "cyan"))
        print(colored("🚀 현재 보트 상태", "yellow", attrs=["bold"]))
        print(colored(separator, "cyan"))

        # 웨이포인트 및 위치 정보
        print(f"🧭 {colored('웨이포인트 번호', 'green')}: {self.WP_k[1].num.data}")
        print(f"📍 {colored('목표 위치', 'green')}: x = {self.WP_k[1].x.data:.3f}, y = {self.WP_k[1].y.data:.3f}")
        print(f"🚢 {colored('현재 위치', 'green')}: x = {self.x_ned:.3f}, y = {self.y_ned:.3f}")

        # 헤딩 및 거리 정보
        print(f"🧭 {colored('현재 헤딩[psi_ned]', 'blue')}: {degrees(self.psi_ned):.2f}°")
        print(f"🎯 {colored('목표 헤딩[target_heading]', 'blue')}: {degrees(self.target_heading):.2f}°")
        print(f"🧭 {colored('목표 각도[psi_diff]', 'yellow')}: {self.target_angle:.4f}°")
        print(f"📏 {colored('목표까지 거리[d_goal]', 'magenta')}: {self.d_goal:.3f} m")

        # 제어 상태
        print(colored("⚙️ 제어 상태:", "yellow", attrs=["bold"]))
        print(f"  🚀 {colored('추진 출력', 'red')}: 좌측 = {self.thruster_p}, 우측 = {self.thruster_s}")

        # 추가 정보
        print(f"🛡️ {colored('도달 가능한 벡터 수', 'cyan')}: {self.non_cross_vector}")
        print(f"🛡️ {colored('벡터 내용', 'cyan')}: {self.detecting_points}")
        print(f"🛡️ {colored('Qorl', 'cyan')}: {self.psi_desire - self.psi_ned}")
        print(f"🧭 {colored('제어각도[control_angle]', 'yellow')}: {self.control_angle:.4f}°")
        print(f"🧭 {colored('최적벡터[psi_desire]', 'blue')}: {self.psi_desire:.2f}°")

        # 선택된 최적 벡터
        direction = "◀ 좌회전" if self.control_angle < 0 else "▶ 우회전"
        print(colored(f"✅ 선택된 벡터: {self.vector_desired:.2f}° {direction}", "green"))


        print(colored(separator, "cyan"))

        
def main():
    rospy.init_node("ship", anonymous=True)
    rate = rospy.Rate(10)
    ship = SHIP()
    
    while not rospy.is_shutdown():
        ship.publish_tf(None)
        ship.publish_scanner_tf()
        ship.ship_run()
        ship.control_publish()
        ship.print_state()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass