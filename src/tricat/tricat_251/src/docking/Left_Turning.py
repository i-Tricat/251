#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2  
import rospy  
from std_msgs.msg import UInt16  
#from image_sensing import Sensing  #이거는 내꺼 따로 돌릴때
from docking.image_sensing import Sensing #이거는 main.py할때

class Turning:
    def __init__(self):
        # 퍼블리셔 초기화
        self.servo_p_pub = rospy.Publisher("/Control/servo_p", UInt16, queue_size=1)  
        self.servo_s_pub = rospy.Publisher("/Control/servo_s", UInt16, queue_size=1)  
        self.thruster_p_pub = rospy.Publisher("/Control/thruster_p", UInt16, queue_size=1)  
        self.thruster_s_pub = rospy.Publisher("/Control/thruster_s", UInt16, queue_size=1)  

        # 명령 값 초기화 (인스턴스 변수로 정의)
        self.servo_p_command = 124  
        self.servo_s_command = 70   
        self.thruster_p_command = 1530  
        self.thruster_s_command = 1530  

    def turn_left(self):
        """로봇을 왼쪽으로 회전시키기 위해 서보 모터와 추력기 명령을 퍼블리시"""
        rospy.loginfo("왼쪽으로 회전")  
        
        # 명령 값 퍼블리시
        self.servo_p_pub.publish(UInt16(self.servo_p_command))  
        self.servo_s_pub.publish(UInt16(self.servo_s_command))  
        self.thruster_p_pub.publish(UInt16(self.thruster_p_command))  
        self.thruster_s_pub.publish(UInt16(self.thruster_s_command))  
        
        # 퍼블리시된 명령 값을 출력하여 디버깅 용도로 사용
        print(f"Publishing Servo P Command: {self.servo_p_command}")
        print(f"Publishing Servo S Command: {self.servo_s_command}")
        print(f"Publishing Thruster P Command: {self.thruster_p_command}")
        print(f"Publishing Thruster S Command: {self.thruster_s_command}")

    def stop(self):
        """로봇을 정지시키기 위해 서보 모터와 추력기 명령을 퍼블리시"""
        rospy.loginfo("정지")  
        
        # 정지 명령 퍼블리시
        self.servo_p_pub.publish(UInt16(data=95))  
        self.servo_s_pub.publish(UInt16(data=95))  
        self.thruster_p_pub.publish(UInt16(data=1500))  
        self.thruster_s_pub.publish(UInt16(data=1500))  

def main():
    rospy.init_node('turning_node', anonymous=True)  
    turning = Turning()  

    
    sensing = Sensing(target_shape='Triangle', color_to_detect='red') 

    # 카메라 초기화
    cap = cv2.VideoCapture(0)  
    if not cap.isOpened():
        rospy.logerr("웹캠을 열 수 없습니다.")  
        raise Exception("웹캠을 열 수 없습니다.")  

    # 카메라 해상도 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)  
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920) 

    rate = rospy.Rate(10)  

    try:
        while not rospy.is_shutdown():
            
            turning.turn_left()

            
            ret, frame = cap.read()  
            if not ret:
                rospy.logerr("프레임을 가져올 수 없습니다.")  
                break
            
            # 프레임의 윗부분 1/4을 잘라내기
            height = frame.shape[0]
            Frame = frame[height // 4:,: ]  # 이미지의 윗부분 1/4만 잘라냄

            # 한 프레임에서 도형 인식 시도
            detected, shape_img = sensing.process_once(Frame)  # 도형 인식 시도
            cv2.imshow('Raw Image', Frame)  # 원본 이미지를 화면에 표시
            cv2.imshow('Detected Shapes', shape_img)  # 감지된 도형 이미지를 화면에 표시

            if detected:
                # 도형이 인식되면  정지
                turning.stop()
                
                # 도형이 인식된 이후에도 지속적인 이미지 처리를 계속함
                sensing.process(cap)
                break  # 도형이 인식되면 루프 종료

            # ESC 키를 누르면 종료
            if cv2.waitKey(1) == 27:
                break

            rate.sleep()  # 설정된 주기에 따라 루프 실행

    except rospy.ROSInterruptException:
        turning.stop()  
    finally:
        cap.release()  
        cv2.destroyAllWindows()  

if __name__ == "__main__":
    main()  
