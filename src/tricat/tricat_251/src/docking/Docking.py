#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy  
from std_msgs.msg import UInt16, String  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge  
import cv2  
import numpy as np  
import time  

# 도형 면적의 임계값, 이 값 이상이면 추적을 멈추도록 설정
STOP_AREA_THRESHOLD = 50000

def contour_points(contour):
    """도형의 외곽선에서 상단 왼쪽과 하단 오른쪽 점을 계산하여 반환"""
    box_left_top = (min(contour[:, 0, 0]), min(contour[:, 0, 1]))  # 외곽선의 상단 왼쪽 점
    box_right_bottom = (max(contour[:, 0, 0]), max(contour[:, 0, 1]))  # 외곽선의 하단 오른쪽 점
    center_col = int((box_left_top[0] + box_right_bottom[0]) / 2)  # 도형의 중심 열 계산
    center_row = int((box_left_top[1] + box_right_bottom[1]) / 2)  # 도형의 중심 행 계산
    return [box_left_top, box_right_bottom], [center_row, center_col]  # 상단 왼쪽, 하단 오른쪽 점 및 중심점 반환

class Docking:
    def __init__(self, target_shape, target_color):
        """Docking 클래스의 초기화 메서드"""
        # ROS 구독자 초기화 (노드 초기화는 main 함수에서 이루어짐)
        rospy.Subscriber('camera/image', Image, self.image_callback)  # 카메라 이미지 메시지 구독

        # ROS 발행자 초기화
        self.servo_p_pub = rospy.Publisher("/Control/servo_p", UInt16, queue_size=1)  
        self.servo_s_pub = rospy.Publisher("/Control/servo_s", UInt16, queue_size=1)  
        self.thruster_p_pub = rospy.Publisher("/Control/thruster_p", UInt16, queue_size=1)  
        self.thruster_s_pub = rospy.Publisher("/Control/thruster_s", UInt16, queue_size=1)  
        self.direction_pub = rospy.Publisher('/direction', String, queue_size=10)  

        
        self.bridge = CvBridge()

        # 변수 초기화
        self.detected_shape = None  # 감지된 도형 정보 저장
        self.image = None  # 카메라에서 받은 이미지를 저장
        self.target_color = target_color  # 목표 색상
        self.target_shape = target_shape  # 목표 도형
        self.shape_center = None  # 감지된 도형의 중심점 저장
        self.tracking = True  # 이미지 추적 상태를 나타내는 플래그

    def shape_info_callback(self, shape_info):
        """도형 정보 콜백 함수, 감지된 도형의 정보를 받아 처리"""
        print("Shape info callback called")
        area, contour, shape_name = shape_info  # 도형 정보에서 면적, 외곽선, 이름을 가져옴
        if shape_name == self.target_shape:  # 감지된 도형이 목표 도형과 일치하는 경우
            _, center_point = contour_points(contour)  # 도형의 중심점 계산
            self.shape_center = center_point  # 중심점 업데이트
            print(f"Detected shape: {shape_name}, Area: {area}, Center: {self.shape_center}")

            if area >= STOP_AREA_THRESHOLD:  # 도형의 면적이 임계값을 넘으면
                rospy.loginfo("면적이 커 추적을 중지합니다.")
                rospy.sleep(1)  # 1초 동안 대기
                # 보트 정렬
                self.servo_p_pub.publish(UInt16(95))  
                self.servo_s_pub.publish(UInt16(95))  
                rospy.loginfo("정렬완료. 앞으로 갈 준비 완료.")
                rospy.sleep(1)  
                start_time = time.time()
                duration = 1  
                servo_p_command = 95
                servo_s_command = 95
                thruster_p_command = 1500
                thruster_s_command = 1500

                # 주어진 시간 동안 앞으로 이동
                while time.time() - start_time < duration:
                    self.servo_p_pub.publish(UInt16(servo_p_command))
                    self.servo_s_pub.publish(UInt16(servo_s_command))
                    self.thruster_p_pub.publish(UInt16(thruster_p_command))
                    self.thruster_s_pub.publish(UInt16(thruster_s_command))
                    rospy.loginfo(f"앞으로 전진.\n"
                                  f"Thruster_P: {thruster_p_command}\n"
                                  f"Thruster_S: {thruster_s_command}\n"
                                  f"Servo_P: {servo_p_command}\n"
                                  f"Servo_S: {servo_s_command}")
                
                self.tracking = False
            else:
                # 도형이 임계값을 넘지 않으면 보트를 제어하여 도형을 추적
                self.control_boat()

    def image_callback(self, data):
        """카메라에서 받은 이미지 메시지를 처리하는 콜백 함수"""
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion error: {e}")  # 이미지 변환 중 오류 발생 시 로그 출력

    def control_boat(self):
        """감지된 도형의 위치에 따라 보트를 제어하는 함수"""
        # 도형 중심과 이미지가 유효하고 추적 상태가 활성화된 경우에만 실행
        if self.shape_center and self.image is not None and self.tracking:
            center_x = self.shape_center[1]  # 감지된 도형의 중심 열 위치

            # 이미지의 중앙과 도형 중심 간의 차이 계산
            line_position = self.image.shape[1] // 2  # 이미지의 중앙 열 위치
            error_x = center_x - line_position  # 에러 계산 (도형 중심과 이미지 중앙의 차이)

            # 디버깅 정보 출력
            print(f"Center X: {center_x}, Line Position: {line_position}, Error X: {error_x}")

            # 제어 로직: 방향 제어를 위한 비례 제어(P 제어)
            k_p = 0.05  # 비례 제어 상수
            servo_p_command = 95 - k_p * error_x  
            servo_s_command = 95 - k_p * error_x  

            # 명령 값을 정수로 변환하고 유효한 범위(0~180)로 제한
            servo_p_command = int(np.clip(servo_p_command, 0, 180))
            servo_s_command = int(np.clip(servo_s_command, 0, 180))
            thruster_p_command = int(1500)  
            thruster_s_command = int(1500)

            # 에러 값에 따라 방향 결정
            if error_x > 0:
                direction = "right"
                rospy.loginfo("Adjusting right")  
            elif error_x < 0:
                direction = "left"
                rospy.loginfo("Adjusting left")  
            else:
                direction = "center"
                rospy.loginfo("On center")  

            # 방향 및 제어 명령 퍼블리시
            self.direction_pub.publish(direction)
            rospy.loginfo(f"Publishing direction: {direction}\n"
                        f"Thruster_P: {thruster_p_command}, Thruster_S: {thruster_s_command}\n"
                        f"Servo_P: {servo_p_command}, Servo_S: {servo_s_command}")

            # 퍼블리시된 명령 값을 출력하여 디버깅 용도로 사용
            print(f"Publishing Servo P Command: {servo_p_command}")
            print(f"Publishing Servo S Command: {servo_s_command}")
            print(f"Publishing Thruster P Command: {thruster_p_command}")
            print(f"Publishing Thruster S Command: {thruster_s_command}")

            # 명령 값을 ROS 퍼블리셔를 통해 실제로 퍼블리시
            self.servo_p_pub.publish(UInt16(servo_p_command))
            self.servo_s_pub.publish(UInt16(servo_s_command))
            self.thruster_p_pub.publish(UInt16(thruster_p_command))
            self.thruster_s_pub.publish(UInt16(thruster_s_command))
        
        
        elif not self.tracking:
            rospy.sleep(1)  # 1초 동안 전진
