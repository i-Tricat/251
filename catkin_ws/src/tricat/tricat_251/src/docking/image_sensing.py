#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2  
import rospy  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge  
import numpy as np  
from docking.Docking import Docking  

# 도형의 면적이 이 임계값보다 크면 추적을 멈추도록 설정
STOP_AREA_THRESHOLD = 500000 



def mean_brightness(img):
    """이미지의 평균 밝기를 조정하여 고정된 값으로 설정"""
    fixed = 100  # 고정된 평균 밝기 값
    m = cv2.mean(img)  # 이미지의 평균 밝기 계산
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv2.add(img, scalar)  # 이미지의 각 채널에 밝기 조정 적용
    return dst
    
def motion_blur_correction(img):
    """모션 블러 보정을 위한 필터 적용"""
    img_blurred = cv2.GaussianBlur(img, (5, 5), 0)  # 가우시안 블러를 사용하여 노이즈 제거
    img_bilateral = cv2.bilateralFilter(img_blurred, 9, 75, 75)  # 양방향 필터를 사용하여 블러 제거 및 가장자리 보존
    return img_bilateral    

def illumination_correction(img):
    """이미지의 조명을 균일화하여 대비를 개선"""
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)  # BGR 이미지를 YUV 색 공간으로 변환
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])  # Y 채널(밝기)에 히스토그램 평활화 적용
    return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)  # 다시 BGR로 변환하여 반환

def apply_white_balance(img):
    """화이트 밸런스를 조정하여 색상을 보다 자연스럽게 보이도록 조정"""
    result = cv2.xphoto.createSimpleWB().balanceWhite(img)  # OpenCV의 화이트 밸런스 기능 사용
    return result

def adjust_gamma(image, gamma=1.0):
    """감마 보정을 통해 이미지의 밝기를 조정"""
    invGamma = 1.0 / gamma  # 감마 값의 역수를 계산
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)  # Look-Up Table(LUT)을 사용하여 감마 보정 적용

def preprocess_image(raw_img, target_color, hsv=True, blur=False, brightness=False):
    """이미지 전처리 함수: 화이트 밸런스, 감마 조정, 조명 보정 등을 적용"""
    img = raw_img.copy()  # 원본 이미지 복사
    img = apply_white_balance(img)  # 화이트 밸런스 적용
    img = adjust_gamma(img, gamma=1.5)  # 감마 보정 적용
    if brightness:
        img = mean_brightness(img)  # 평균 밝기 조정
    img = illumination_correction(img)  # 조명 보정 적용
    if blur:
        img = cv2.GaussianBlur(img, (5, 5), 0)  # 가우시안 블러링 적용
    if hsv and target_color != 'black':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 색상 검출을 위해 HSV 색 공간으로 변환
    return img

def select_color(img, color_range, target_color, thresh=50):
    """이미지에서 특정 색상을 검출하기 위해 이진 마스크 이미지를 생성"""
    if target_color == 'black':
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 이미지를 그레이스케일로 변환
        _, frame_color_binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)  # 이진화하여 검정색 영역 검출
        return frame_color_binary
    else:
        frame_color_binary = cv2.inRange(img, np.array(color_range[0]), np.array(color_range[1]))  # 주어진 색상 범위 내의 색상만 남기는 이진 마스크 생성
        return frame_color_binary

def contour_points(contour):
    """도형의 외곽선에서 상단 왼쪽과 하단 오른쪽 점을 계산하여 반환"""
    box_left_top = (min(contour[:, 0, 0]), min(contour[:, 0, 1]))  # 외곽선의 상단 왼쪽 점
    box_right_bottom = (max(contour[:, 0, 0]), max(contour[:, 0, 1]))  # 외곽선의 하단 오른쪽 점
    center_col = int((box_left_top[0] + box_right_bottom[0]) / 2)  # 외곽선의 중심 열 계산
    center_row = int((box_left_top[1] + box_right_bottom[1]) / 2)  # 외곽선의 중심 행 계산
    return [box_left_top, box_right_bottom], [center_row, center_col]  # 상단 왼쪽, 하단 오른쪽 점 및 중심점 반환

def draw_mark(window, contour, vertices, area, box_points, center_point, shape_name, is_target=False):
    """감지된 도형의 외곽선을 그리고 정보를 화면에 표시"""
    color = (0, 255, 0) if is_target else (15, 219, 250)  # 타겟 도형이면 초록색, 아니면 다른 색
    caption = "{} Area: {:.2f}".format(shape_name, area)  # 도형 이름과 면적을 포함한 캡션
    window = cv2.drawContours(window, [contour], -1, color, -1)  # 도형 외곽선을 그림
    window = cv2.rectangle(window, box_points[0], box_points[1], color, 1)  # 도형 주위에 사각형 그리기
    window = cv2.circle(window, (center_point[1], center_point[0]), 2, (0, 0, 255), 2)  # 도형 중심에 점을 찍음
    window = cv2.putText(window, caption, (box_points[0][0], box_points[0][1] - 10), cv2.FONT_HERSHEY_PLAIN, 1, color, 1, cv2.LINE_AA)

    # 화면의 중앙선 그리기
    line_position = window.shape[1] // 2  # 중앙선 위치 계산
    window = cv2.line(window, (line_position, 0), (line_position, window.shape[0]), (255, 0, 0), 2)  # 수직 중앙선 그리기
    window = cv2.line(window, (center_point[1], 0), (center_point[1], window.shape[0]), (0, 0, 255), 2)  # 도형 중심점에서의 수직선 그리기

    return window

def detect_target(img, mark_detect_area, target_detect_area, target_shape):
    """이미지에서 특정 도형을 감지하고 감지된 도형을 반환"""
    detected_shapes = []  # 감지된 도형을 저장할 리스트
    morph_kernel_close = np.ones((9, 9), np.uint8)  # 모폴로지 연산에 사용할 커널 (닫기 연산용)
    morph_kernel_open = np.ones((5, 5), np.uint8)  # 모폴로지 연산에 사용할 커널 (열기 연산용)
    morph_kernel_small = np.ones((3, 3), np.uint8)  # 모폴로지 연산에 사용할 작은 커널

    # 모폴로지 변환을 적용하여 노이즈 제거 및 도형 윤곽 강조
    morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel_close)  # 닫기 연산
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, morph_kernel_open)  # 열기 연산
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, morph_kernel_small)  # 작은 열기 연산

    # 외곽선을 찾아 도형을 감지
    contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    shape = np.zeros((morph.shape[0], morph.shape[1], 3), dtype=np.uint8)  # 도형을 그릴 빈 캔버스 생성

    for contour in contours:
        peri = cv2.arcLength(contour, True)  # 외곽선의 길이를 계산
        approx = cv2.approxPolyDP(contour, 0.027 * peri, True)  # 도형을 근사화
        area = cv2.contourArea(approx)  # 근사화된 도형의 면적 계산
        if area < mark_detect_area:
            continue  # 면적이 임계값보다 작으면 무시

        vertex_num = len(approx)  # 도형의 꼭짓점 수
        detected = False
        shape_name = "Other"  # 기본 도형 이름은 "Other"

        if peri == 0:
            continue

        # 원형도(circularity) 계산
        circularity = 4 * np.pi * (area / (peri * peri))
        if circularity < 0.4:  # 원형도가 0.42 미만이면 무시 (Cross 인식 기준)
            continue

        # 도형의 회전 및 기울기에 대한 견고성 강화 (최소 외접 사각형 사용)
        rect = cv2.minAreaRect(contour)  # 최소 외접 사각형 계산
        box = cv2.boxPoints(rect)  # 사각형의 4개 꼭짓점 좌표 계산
        box = np.int0(box)  # 정수로 변환

        # 도형의 형태를 꼭짓점 수와 면적에 따라 판별
        if target_shape == "Triangle" and vertex_num == 3 and area >= target_detect_area:
            shape_name = "Triangle"
            detected = True
        elif target_shape == "Rectangle" and vertex_num == 4 and area >= target_detect_area:
            shape_name = "Rectangle"
            detected = True
        elif target_shape == "Star" and vertex_num == 10 and area >= target_detect_area:
            shape_name = "Star"
            detected = True
        elif target_shape == "Cross" and vertex_num == 12 and area >= target_detect_area:
            shape_name = "Cross"
            detected = True
        elif target_shape == "Circle" and vertex_num > 5:
            ellipse = cv2.fitEllipse(contour)  # 원형 도형에 타원 맞추기
            (center, axes, orientation) = ellipse  # 타원의 중심, 축, 회전각 계산
            major_axis_length = max(axes)  # 큰 축 길이
            minor_axis_length = min(axes)  # 작은 축 길이
            if area >= target_detect_area:
                aspect_ratio = major_axis_length / minor_axis_length  # 비율 계산
                if aspect_ratio > 0.9:  # 원형의 기준을 0.9 이상으로 설정
                    shape_name = "Circle"
                    detected = True

        if detected:
            detected_shapes.append((area, contour, shape_name))  # 감지된 도형 리스트에 추가

    # 감지된 도형 중 가장 큰 도형만 선택하여 반환
    detected_shapes = sorted(detected_shapes, key=lambda x: x[0], reverse=True)[:1]
    for area, contour, shape_name in detected_shapes:
        box_points, center_point = contour_points(contour)  # 도형의 꼭짓점과 중심점 계산
        shape = draw_mark(shape, contour, len(contour), area, box_points, center_point, shape_name, True)  # 도형을 화면에 그리기

    return detected_shapes, shape # 감지된 도형과 그린 도형 이미지를 반환

def define_color_range(hue_min, hue_max, sat_min=50, sat_max=255, val_min=50, val_max=255):
    """HSV 색상 범위를 정의하여 특정 색상을 선택"""
    lower = np.array([hue_min, sat_min, val_min])  # 하한 값
    upper = np.array([hue_max, sat_max, val_max])  # 상한 값
    return lower, upper

class Sensing:
    """목표 도형과 색상을 감지하는 클래스"""
    def __init__(self, target_shape='Cross', color_to_detect='blue'):
        self.target_shape = target_shape  # 감지할 도형 형태
        self.color_to_detect = color_to_detect  # 감지할 색상
        self.bridge = CvBridge()  # ROS와 OpenCV 간의 이미지 변환을 처리하는 브릿지
        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)  # ROS 이미지 퍼블리셔
        self.color_ranges = {
            'red': define_color_range(170, 180),
            'blue': define_color_range(90, 110),
            'yellow': define_color_range(15, 30),
            'orange': define_color_range(0, 14),
            'green': define_color_range(60, 85),
            'purple': define_color_range(125, 155)
        }  # 색상 범위 딕셔너리
        self.docking = Docking(self.target_shape, self.color_to_detect)  # Docking 클래스 초기화

    def process_once(self, raw_img):
        """한 프레임을 처리하고 도형이 인식되면 True를 반환"""
        preprocessed_img = preprocess_image(raw_img, self.color_to_detect)  # 이미지 전처리

        mask = select_color(preprocessed_img, self.color_ranges.get(self.color_to_detect, []), self.color_to_detect)  # 색상 선택
        targets, shape_img = detect_target(mask, 1000, 2000, self.target_shape)  # 도형 감지

        if targets:
            largest_shape = max(targets, key=lambda x: x[0])  # 가장 큰 도형 선택
            # self.docking.shape_info_callback(largest_shape)  # Docking 클래스에 도형 정보 전달
            img_msg = self.bridge.cv2_to_imgmsg(raw_img, encoding="bgr8")  # OpenCV 이미지를 ROS 이미지로 변환
            self.image_pub.publish(img_msg)  # ROS 토픽으로 이미지 퍼블리시

            return True, shape_img, largest_shape  # 도형이 인식되면 True와 도형 이미지를 반환
            # return True, shape_img
        
        largest_shape = None
        return False, raw_img, largest_shape  # 도형이 인식되지 않으면 False와 원본 이미지를 반환
        # return False, raw_img

    def process(self, cap):
        """카메라 프레임을 지속적으로 처리하여 도형 감지"""
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            ret, frame = cap.read()  
            if not ret:
                rospy.logerr("프레임을 가져올 수 없습니다.")
                break
            height = frame.shape[0]
            Frame = frame[height // 4:, :]  # 프레임의 윗부분 1/4을 잘라냄

            detected, shape_img = self.process_once(Frame)  # 한 프레임을 처리하여 도형 감지 시도
            cv2.imshow('Raw Image', Frame)  # 원본 이미지를 화면에 표시
            cv2.imshow('Detected Shapes', shape_img)  # 검출된 도형 이미지를 화면에 표시

            if cv2.waitKey(1) == 27:  # ESC 키를 누르면 종료
                break

            rate.sleep()

        cv2.destroyAllWindows()  # 모든 창 닫기

# HSV 값 (색상 범위를 정의하는 데 사용)
#1. red: 170,180,120
#2. black: 0,0,0
#3. blue: 104,103,90
#4. green: 78,100,90 
#5. orange: 2,150,100
#6. yellow: 22,80,100
