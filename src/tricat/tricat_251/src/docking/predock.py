#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
import numpy as np
from std_msgs.msg import UInt16
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import time
# from visualizer import Visualizer


class PredockCal():
    def __init__(self):
        self.front_angle_range = rospy.get_param('front_angle_range', (-7, 1))  # 정면 각도 범위 설정
        self.right_angle_range = rospy.get_param('right_angle_range', (-120, -60))  # 오른쪽 각도 범위 설정
        self.left_angle_range = rospy.get_param('left_angle_range', (60, 120))  # 왼쪽 각도 범위

        self.front_wall_points = []
        self.right_wall_points = []
        self.left_wall_points = []

        self.front_distance = None
        self.right_distance = None
        self.left_distance = None
        self.front_wall_angle = None
        self.right_wall_angle = None
        self.left_wall_angle = None

        self.point_cloud_data = None
        rospy.Timer(rospy.Duration(0.1), self.process_point_cloud)

    def get_point_cloud_data(self, data):
        self.point_cloud_data = data

    # 포인트 클라우드 데이터 처리
    def process_point_cloud(self, event=None):
        if self.point_cloud_data is None:
            print("No point cloud data received yet.")
            # rospy.logwarn("No point cloud data received yet.")
            return

        try:
            # 포인트 클라우드 데이터를 변환
            transformed_points = self.transform_points(self.point_cloud_data)

            # 각도 범위별로 포인트 필터링
            filtered_front_points = self.filter_points(transformed_points, self.front_angle_range)
            filtered_right_points = self.filter_points(transformed_points, self.right_angle_range)
            filtered_left_points = self.filter_points(transformed_points, self.left_angle_range)

            # 클러스터링하여 벽 점들 확인
            self.front_wall_points = self.cluster_points(filtered_front_points)
            self.right_wall_points = self.cluster_points(filtered_right_points)
            self.left_wall_points = self.cluster_points(filtered_left_points)

            # 정면 거리 및 각도 계산
            if self.front_wall_points:
                self.front_distance = self.calculate_distance(self.front_wall_points)
                left_point, right_point = self.calculate_wall_endpoints(self.front_wall_points)
                self.front_wall_angle = self.calculate_wall_angle(left_point, right_point)

            # 오른쪽 거리 및 각도 계산
            if self.right_wall_points:
                self.right_distance = self.calculate_distance(self.right_wall_points)
                left_point, right_point = self.calculate_wall_endpoints(self.right_wall_points)
                self.right_wall_angle = self.calculate_wall_angle(left_point, right_point)

            # 왼쪽 거리 및 각도 계산
            if self.left_wall_points:
                self.left_distance = self.calculate_distance(self.left_wall_points)
                left_point, right_point = self.calculate_wall_endpoints(self.left_wall_points)
                self.left_wall_angle = self.calculate_wall_angle(left_point, right_point)

        except Exception as e:
            rospy.logerr(f"Error in process_point_cloud {e}")

    # 포인트 클라우드 데이터를 처리하여 포인트 변환
    def transform_points(self, data):
        points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return []
        transformed_points = [(point[0], point[1]) for point in points if point[2] <= 0]
        return transformed_points

    # 특정 각도 범위 내의 포인트 필터링
    def filter_points(self, points, angle_range):
        filtered_points = []
        for point in points:
            angle = math.degrees(math.atan2(point[1], point[0]))
            if angle_range[0] <= angle <= angle_range[1]:
                filtered_points.append(point)
        return filtered_points

    # DBSCAN을 사용한 포인트 클러스터링
    def cluster_points(self, points, eps=0.1, min_samples=10, outlier_threshold=0.0001):
        if not points:
            return []
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_

        if len(set(labels)) == 1 and -1 in labels:
            return []  # 클러스터가 없는 경우 빈 리스트 반환

        unique_labels = set(labels)
        max_label = max(unique_labels, key=lambda label: list(labels).count(label))
        wall_points = [points[i] for i in range(len(points)) if labels[i] == max_label]

        if wall_points:
            left_point, right_point = self.calculate_wall_endpoints(wall_points)
            wall_length = math.sqrt((right_point[0] - left_point[0]) ** 2 + (right_point[1] - left_point[1]) ** 2)
            if wall_length < 0.1:
                return []

            # 끝점에서 너무 떨어진 점들 제거
            filtered_wall_points = []
            for point in wall_points:
                dist_to_line = abs((right_point[1] - left_point[1]) * point[0] -
                                   (right_point[0] - left_point[0]) * point[1] +
                                   right_point[0] * left_point[1] - right_point[1] * left_point[0]) / wall_length
                if dist_to_line <= outlier_threshold:
                    filtered_wall_points.append(point)
            return filtered_wall_points

        return wall_points

    # 포인트 집합의 중심점으로부터의 거리 계산
    def calculate_distance(self, points):
        if not points:
            return None
        center_x = np.mean([point[0] for point in points])
        center_y = np.mean([point[1] for point in points])
        distance = math.sqrt(center_x**2 + center_y**2)
        return distance

    # 벽의 양 끝점 계산
    def calculate_wall_endpoints(self, points):
        center_x = np.mean([point[0] for point in points])
        center_y = np.mean([point[1] for point in points])
        max_dist_left = -1
        max_dist_right = -1
        left_point = None
        right_point = None
        for point in points:
            dist = math.sqrt((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2)
            if point[0] < center_x and dist > max_dist_left:
                left_point = point
                max_dist_left = dist
            elif point[0] > center_x and dist > max_dist_right:
                right_point = point
                max_dist_right = dist
        return left_point, right_point

    # 벽의 각도 계산
    def calculate_wall_angle(self, left_point, right_point):
        delta_y = right_point[1] - left_point[1]
        delta_x = right_point[0] - left_point[0]
        angle = math.atan2(delta_y, delta_x)
        return int(math.degrees(angle))

class STController:
    def __init__(self):
        # 파라미터 서버에서 초기 파라미터 값을 로드
        self.servo_p = rospy.get_param('servo_p_stop', 95)
        self.servo_s = rospy.get_param('servo_s_stop', 95)
        self.thruster_p = rospy.get_param('thruster_p_stop', 1500)
        self.thruster_s = rospy.get_param('thruster_s_stop', 1500)
        self.outinfo = ""

    def stop(self):
        # rospy.loginfo("정지 명령 실행")
        self.outinfo = "정지 명령 실행"
        self.servo_p = rospy.get_param('servo_p_stop', 95)
        self.servo_s = rospy.get_param('servo_s_stop', 95)
        self.thruster_p = rospy.get_param('thruster_p_stop', 1500)
        self.thruster_s = rospy.get_param('thruster_s_stop', 1500)

    def turn_left(self):
        # rospy.loginfo("왼쪽으로 회전")
        self.outinfo = "왼쪽으로 회전"
        self.servo_p = rospy.get_param('servo_p_turn_left', 125)
        self.servo_s = rospy.get_param('servo_s_turn_left', 70)
        self.thruster_p = rospy.get_param('thruster_p_turn_left', 1500)
        self.thruster_s = rospy.get_param('thruster_s_turn_left', 1500)

    def drift_left(self, right_wall_angle, front_distance):
        self.outinfo = "왼쪽으로 드리프트"
        angle_adjustment_max = rospy.get_param('angle_adjustment_max', 15)
        distance_adjustment_max = rospy.get_param('distance_adjustment_max', 70)  # 최대 거리 보정 값

        # 각도에 따른 서보 조정
        if right_wall_angle is not None:
            angle_adjustment = min(max(abs(right_wall_angle), 0), 90) // 6 + 1
            angle_adjustment = min(angle_adjustment, angle_adjustment_max)
            self.servo_p = rospy.get_param('servo_p', 95) + angle_adjustment
            self.servo_s = rospy.get_param('servo_s', 95) + angle_adjustment

        # 거리에 따른 쓰러스터 조정
        if front_distance is not None:
            distance_adjustment = min(max(int(front_distance), 0), 7) * 10
            distance_adjustment = min(distance_adjustment, distance_adjustment_max)
            self.thruster_p = rospy.get_param('thruster_p_drift_right', 1570) - distance_adjustment
            self.thruster_s = rospy.get_param('thruster_s_drift_right', 1570) - distance_adjustment

    def turn_right(self):
        # rospy.loginfo("오른쪽으로 회전")
        self.outinfo = "오른쪽으로 회전"
        self.servo_p = rospy.get_param('servo_p_turn_right', 110)
        self.servo_s = rospy.get_param('servo_s_turn_right', 75)
        self.thruster_p = rospy.get_param('thruster_p_turn_right', 1500)
        self.thruster_s = rospy.get_param('thruster_s_turn_right', 1500)

    def drift_right(self, right_wall_angle, front_distance):
        self.outinfo = "오른쪽으로 드리프트"
        angle_adjustment_max = rospy.get_param('angle_adjustment_max', 15)
        distance_adjustment_max = rospy.get_param('distance_adjustment_max', 70)  # 최대 거리 보정 값

        # 각도에 따른 서보 조정
        if right_wall_angle is not None:
            angle_adjustment = min(max(abs(right_wall_angle), 0), 90) // 6 + 1
            angle_adjustment = min(angle_adjustment, angle_adjustment_max)
            self.servo_p = rospy.get_param('servo_p', 95) - angle_adjustment
            self.servo_s = rospy.get_param('servo_s', 95) - angle_adjustment

        # 거리에 따른 쓰러스터 조정
        if front_distance is not None:
            distance_adjustment = min(max(int(front_distance), 0), 7) * 10
            distance_adjustment = min(distance_adjustment, distance_adjustment_max)
            self.thruster_p = rospy.get_param('thruster_p_drift_right', 1570) - distance_adjustment
            self.thruster_s = rospy.get_param('thruster_s_drift_right', 1570) - distance_adjustment

    def go(self, front_distance):
        self.outinfo = "직진"
        self.servo_p = rospy.get_param('servo_p_go', 95)
        self.servo_s = rospy.get_param('servo_s_go', 95)
        distance_adjustment_max = rospy.get_param('distance_adjustment_max', 70)  # 최대 거리 보정 값
        # 거리에 따른 쓰러스터 조정
        if front_distance is not None:
            distance_adjustment = min(max(int(front_distance), 0), 7) * 10
            distance_adjustment = min(distance_adjustment, distance_adjustment_max)
            self.thruster_p = rospy.get_param('thruster_p_drift_right', 1570) - distance_adjustment
            self.thruster_s = rospy.get_param('thruster_s_drift_right', 1570) - distance_adjustment


class PreDockController(STController):
    def __init__(self):
        # 초기 상태 설정
        super().__init__() 
        self.state = "front"
        self.front_distance = None
        self.right_distance = None
        self.left_distance = None
        self.front_wall_angle = None
        self.right_wall_angle = None
        self.left_wall_angle = None
        self.r_p_distance = None
        self.turning_left = False
        self.turned_in_right = False  # 왼쪽 회전 중인지 여부
        # 상태 초기화
        self.right_start_time = None
        self.turn_start_time = None
    

        rospy.Timer(rospy.Duration(0.1), self.run)  # 0.1초 주기로 호출
        # rospy.Timer(rospy.Duration(0.1), self.state_print)  # 0.1초 주기로 호출

    def get_info(self, front_distance, right_distance, left_distance, front_wall_angle, right_wall_angle, left_wall_angle):
        self.front_distance = front_distance
        self.right_distance = right_distance
        self.left_distance = left_distance
        self.front_wall_angle = front_wall_angle
        self.right_wall_angle = right_wall_angle
        self.left_wall_angle = left_wall_angle

        if self.right_distance is None:
            self.right_distance = 0.0  # 또는 적절한 기본값 설정
        if self.right_wall_angle is None:
            self.right_wall_angle = 0.0
        else:
            self.r_p_distance = self.right_distance * math.cos(math.radians(self.right_wall_angle))

    # 상태에 따른 동작 제어
    def front(self):
        self.state = "front"
        front_distance = self.front_distance
        right_wall_angle = self.right_wall_angle
        r_p_distance = self.r_p_distance
        front_distance_1 = rospy.get_param('front_distance_1', 2.9)
        r_p_11 = rospy.get_param('r_p_11', 2.0)
        r_p_12 = rospy.get_param('r_p_12', 3.0)
        turn_time = rospy.get_param('turn_time', 2.5)
        # 1570 => 2.5초 Turn
        #  
        if self.turning_left:  # 이미 회전 중이라면 회전 유지
            if time.time() - self.turn_start_time < turn_time: 
                self.turn_left()  # 회전 유지
            else:
                self.finish_left_turn(None)  # 회전 종료
            return
        
        # 1570 => front = 2.9, r_p = 1.5~4.0
        if (front_distance is not None and front_distance > front_distance_1) and (r_p_distance is not None and r_p_distance > r_p_12):
            if right_wall_angle is not None:
                if right_wall_angle > 0:
                    self.go()
                elif right_wall_angle < 0:
                    self.drift_right(right_wall_angle)

        elif (front_distance is not None and front_distance > front_distance_1) and (r_p_distance is not None and r_p_distance < r_p_11):
            if right_wall_angle is not None:
                if right_wall_angle > 0:
                    self.drift_left(right_wall_angle)
                elif right_wall_angle < 0:
                    self.go()

        elif (front_distance is not None and front_distance > front_distance_1) and (r_p_distance is not None and r_p_11 < r_p_distance < r_p_12):
            if right_wall_angle is not None:
                if right_wall_angle < 0:
                    self.drift_right(right_wall_angle)
                elif right_wall_angle > 0:
                    self.drift_left(right_wall_angle)
        
        elif front_distance is not None and front_distance < 2.9:
            self.turning_left = True  # 회전 상태 활성화
            self.turn_left()
            self.turn_start_time = time.time()
        
    def finish_left_turn(self, event):
        rospy.loginfo("Finished left turn, switching to 'right' state.")
        self.turning_left = False  # 회전 상태 비활성화
        self.state = "right"

    def right(self):
        self.state = "right"
        # rospy.loginfo("보트가 오른쪽 벽을 따라 진행 중입니다.")
        front_distance = self.front_distance
        front_wall_angle = self.front_wall_angle
        right_distance = self.right_distance
        right_wall_angle = self.right_wall_angle
        r_p_distance = self.r_p_distance
        front_distance_2 = rospy.get_param('front_distance_2', 2.9)
        r_p_21 = rospy.get_param('r_p_21', 2.0)
        r_p_22 = rospy.get_param('r_p_22', 3.0)
        turn_time = rospy.get_param('turn_time', 2.5)
        wait_time = rospy.get_param('wait_time', 4.0)
        # 1570 => 2.5초 turn
        # 1570 => front = 1.5, r_p = 2.5
        if self.turning_left:  # 이미 회전 중이라면 회전 유지
            if time.time() - self.turn_start_time < turn_time:
                self.turn_left()  # 회전 유지
            else:
                self.finish_left_turn(None)  # 회전 종료
            return

        if front_distance is not None and front_distance < front_distance_2:
            if front_wall_angle is not None and front_wall_angle < 0:
                self.turning_left = True
                self.turn_left()
                self.turn_start_time = time.time()
                self.turned_in_right = True

            elif front_wall_angle is not None and front_wall_angle > 0:
                self.turning_left = True
                self.turn_start_time = time.time()
                self.turn_left()
                self.turned_in_right = True
            

        if (front_distance is not None and front_distance > front_distance_2) and (r_p_distance is not None and r_p_distance > r_p_22):
            if right_wall_angle is not None:
                if right_wall_angle >= 0:
                    self.go()
                elif right_wall_angle < 0:
                    self.drift_right(right_wall_angle)

        if (front_distance is not None and front_distance > front_distance_2) and (r_p_distance is not None and r_p_distance < r_p_21):
            if right_wall_angle is not None:
                if right_wall_angle > 0:
                    self.drift_left(right_wall_angle)
                elif right_wall_angle <= 0:
                    self.go()

        if (front_distance is not None and front_distance > front_distance_2) and (r_p_distance is not None and r_p_21 < r_p_distance < r_p_22):
            if right_wall_angle is not None:
                if right_wall_angle <= 0:
                    self.drift_right(right_wall_angle)
                elif right_wall_angle > 0:
                    self.drift_left(right_wall_angle)

        # 타이머 시작
        if self.right_start_time is None:
            self.right_start_time = rospy.get_time()

        current_time = rospy.get_time()

        # 1570 => 4.0초 정도면 정렬될듯
        if current_time - self.right_start_time > wait_time:
            self.state = "left"
            self.turned_in_right = False
            self.right_start_time = None

    def left(self):
        self.state = "left"
        # rospy.loginfo("마지막 경우")
        front_distance = self.front_distance
        right_distance = self.right_distance
        right_wall_angle = self.right_wall_angle
        r_p_distance = self.r_p_distance
        front_distance_3 = rospy.get_param('front_distance_3', 5.5)
        r_p_31 = rospy.get_param('r_p_31', 1.3)
        r_p_32 = rospy.get_param('r_p_32', 1.7)

        if front_distance is not None and front_distance < front_distance_3:
            self.state = "done"
        
        if (front_distance is not None and front_distance > front_distance_3) and (r_p_distance is not None and r_p_distance > r_p_32):
            if right_wall_angle is not None:
                if right_wall_angle >= 0:
                    self.go()
                elif right_wall_angle < 0:
                    self.drift_right(right_wall_angle)

        if (front_distance is not None and front_distance > front_distance_3) and (r_p_distance is not None and r_p_distance < r_p_31):
            if right_wall_angle is not None:
                if right_wall_angle > 0:
                    self.drift_left(right_wall_angle)
                elif right_wall_angle <= 0:
                    self.go()

        if (front_distance is not None and front_distance > front_distance_3) and (r_p_distance is not None and r_p_31 < r_p_distance < r_p_32):
            if right_wall_angle is not None:
                if right_wall_angle <= 0:
                    self.drift_right(right_wall_angle)
                elif right_wall_angle > 0:
                    self.drift_left(right_wall_angle)

    # 루프 실행
    def run(self, event=None):
        if self.state == "front":
            self.front()
        elif self.state == "right":
            self.right()
        elif self.state == "left":
            self.left()
        elif self.state == "done":
            self.stop()

    def state_print(self, event=None):
        # 상태 정보 출력
        print("---------------------------------\n"
            f"State: {self.state}\n"
            f"Front distance: {self.front_distance}\n"
            f"Right distance: {self.right_distance}\n"
            f"Front wall angle: {self.front_wall_angle}\n"
            f"Right wall angle: {self.right_wall_angle}\n"
            f"r_p distance: {self.r_p_distance}\n"
            f"control: {self.outinfo}")


#################################################################################
#                                 TEST                                          #
#################################################################################
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import UInt16
# from tricat_msgs.msg import Control

# def point_cloud_callback(data, predock_cal):
#     predock_cal.get_point_cloud_data(data)

# def main():
#     # ROS 노드 초기화
    
#     rospy.init_node('predock_node', anonymous=True)
#     rate = rospy.Rate(10)
#     # PredockCal 및 PreDockController 인스턴스 생성
#     predock_cal = PredockCal()
#     predock_controller = PreDockController()

#     rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback, predock_cal)

#     control_pub = rospy.Publisher("/Control", Control, queue_size=10)
#     control_msg = Control()

#     while not rospy.is_shutdown():
  
#         predock_cal.process_point_cloud()
#         # PredockCal에서 계산된 정보를 PreDockController에 전달
#         predock_controller.get_info(predock_cal.front_distance,
#                                     predock_cal.right_distance,
#                                     predock_cal.left_distance,
#                                     predock_cal.front_wall_angle,
#                                     predock_cal.right_wall_angle,
#                                     predock_cal.left_wall_angle)
#         predock_controller.run()
#         predock_controller.state_print()

#         control_msg.servo_p = UInt16(predock_controller.servo_p)
#         control_msg.servo_s = UInt16(predock_controller.servo_s)
#         control_msg.thruster_p = UInt16(predock_controller.thruster_p)
#         control_msg.thruster_s = UInt16(predock_controller.thruster_s)
#         #control_msg.thruster_p = UInt16(1510)
#         #control_msg.thruster_s = UInt16(1510)
#         control_pub.publish(control_msg)

#         rate.sleep()


# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass