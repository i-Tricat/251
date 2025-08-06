#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import rospy
from ship.ship import ship
from docking.image_sensing import *

import numpy as np
import time

def initialize():
    os = ship()
    rospy.init_node(os.ship_name, anonymous=False)
    rate = rospy.Rate(10)  # 10 Hz
    
    video_num = rospy.get_param('video_num', 1)
    cap = cv2.VideoCapture(0)  
    if not cap.isOpened():
        rospy.logerr("웹캠을 열 수 없습니다.")
        return None, None

    # 카메라 해상도 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)  
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2560) 

    target_shape = rospy.get_param("target_shape", 'Cross')
    color_to_detect = rospy.get_param("color_to_detect", 'blue')
    sensing = Sensing(target_shape, color_to_detect)

    return os, rate, cap, sensing

def docking_process(os, cap, sensing, stop_area_threshold, aline):
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("프레임을 가져올 수 없습니다.")
        return False

    # 프레임의 윗부분 1/4 잘라내기
    height = frame.shape[0]
    cropped_frame = frame[height // 4:, :]  # 윗부분 1/4 잘라내기

    # 도형 인식 시도
    detected, shape_img, largest_shape = sensing.process_once(cropped_frame)
    cv2.imshow('Raw Image', cropped_frame)  # 잘라낸 원본 이미지 표시
    cv2.imshow('Detected Shapes', shape_img)  # 검출된 도형 이미지 표시
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

    if not detected and not aline:  # 인식이 안되면 계속 왼쪽으로 회전
        rospy.loginfo("왼 회전 중")
        os.servo_p = 120
        os.servo_s = 120
        os.thruster_p = 1520
        os.thruster_s = 1520
        return True

    if detected:
        area, contour, _ = largest_shape
        _, shape_center = contour_points(contour)

        if area < stop_area_threshold:
            if shape_center and cropped_frame is not None:
                center_x = shape_center[1]  # 도형의 중심선
                line_position = cropped_frame.shape[1] // 2  # 카메라 중앙선
                error_x = center_x - line_position

                k_p = 0.03  # 고정값

                os.servo_p = int(np.clip(95 - k_p * error_x, 0, 180))
                os.servo_s = int(np.clip(95 - k_p * error_x, 0, 180))
                os.thruster_p = 1550
                os.thruster_s = 1550
        else:
            if not aline:
                rospy.loginfo("정렬완료. 앞으로 갈 준비 완료.")
                rospy.sleep(1)  
                start_time = time.time()
                duration = 1
                os.servo_p = 95
                os.servo_s = 95
                os.thruster_p = 1550
                os.thruster_s = 1550
                rospy.loginfo("전진중...")

                while time.time() - start_time < duration:
                    os.control_publish()
                    #rospy.loginfo(f"Time elapsed: {time.time() - start_time}")

                rospy.loginfo("전진완료.")
                aline = True
                return False
    return True

def main_loop(os, cap, sensing, rate):
    check = False
    aline = False
    num = 10
    stop_area_threshold = 30000
    tracking = True
    thruster = 1600
    thruster_steps = np.linspace(thruster, 1450, 50).tolist()
    current_thruster = 1600

    os.wp_manager.wp_client()

    while not rospy.is_shutdown():
        os.WP_k = os.wp_manager.manage(os.x_ned, os.y_ned)
        os.d_goal = os.wp_manager.d_goal
        
        if os.WP_k[0].num.data == num:
            if not check and os.control_mode != "PreDocking" and os.control_mode != "Docking":
                os.control_mode = "PreDocking"
                os.change_controller("predockcontroller")
                os.predock_run()
                os.predockcontroller.state_print()

            elif not check and os.control_mode == "PreDocking":
                if os.predockcontroller.state == "done":
                    check = True
                    os.control_mode = "Docking"
                    os.change_controller("dockcontroller")
                else:
                    os.predock_run()
                    os.predockcontroller.state_print()

            elif check and os.control_mode == "Docking":
                tracking = docking_process(os, cap, sensing, stop_area_threshold, aline)
                if not tracking:
                    break

        elif os.WP_k[0].num.data == num and not tracking:
            os.change_controller("headingAutopilot")
            os.autopilot_run()

        else:
            os.control_mode = "headingAutopilot"
            os.autopilot_run()

            if os.d_goal <= os.WP_k[1].range.data * 1.5:
                if os.U == 0:
                    current_thruster = 1500
                elif len(thruster_steps) == 1:
                    current_thruster = int(thruster_steps[0])
                else:
                    current_thruster = int(thruster_steps.pop(0))
            else:
                thruster_steps = np.linspace(thruster, 1450, 10).tolist()
                current_thruster = 1600

            os.thruster_s = current_thruster
            os.thruster_p = current_thruster

            os.print_state()

        # Land test mode (Don't use thruster)
        # os.thruster_p = 1600
        # os.thruster_s = 1600
        
        os.control_publish()
        rate.sleep()

    cap.release()  
    cv2.destroyAllWindows()

def main():
    os, rate, cap, sensing = initialize()
    if os and cap and sensing:
        main_loop(os, cap, sensing, rate)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass