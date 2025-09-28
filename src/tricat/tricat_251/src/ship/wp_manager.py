#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy  # ROS 파이썬 라이브러리
from tricat_msgs.srv import WaypointService, WaypointServiceResponse  # 사용자 정의 서비스 메시지 가져오기
from math import hypot  # 두 지점 간 거리 계산을 위한 함수

class WpManager():
    def __init__(self):
        '''
        WpManager 클래스 초기화 메서드
        - WP_data: 전체 웨이포인트 데이터 리스트
        - WP_k: 현재 진행 중인 두 개의 웨이포인트 저장 리스트 (현재 지점과 목표 지점)
        - num_k: 현재 처리 중인 웨이포인트 인덱스
        - d_goal: 현재 위치와 목표 웨이포인트 간 거리
        - ch: 웨이포인트 갱신 여부 플래그
        '''
        self.WP_data = []  # 전체 웨이포인트 데이터 저장
        self.WP_k = []     # 현재 및 다음 웨이포인트 저장
        self.num_k = 0     # 현재 처리 중인 웨이포인트 인덱스
        self.d_goal = 0    # 현재 위치와 목표 지점 간 거리
        self.ch = False    # 웨이포인트 갱신 여부 확인 플래그

    def wp_client(self):
        '''
        ROS 서비스 서버에서 웨이포인트 데이터를 요청하고 받아오는 메서드
        - 'get_waypoints' 서비스를 호출하여 웨이포인트 목록을 받아옴
        - 받아온 웨이포인트는 self.WP_data에 저장됨
        '''
        rospy.wait_for_service('get_waypoints')  # 서비스가 사용할 준비가 될 때까지 대기
        try:
            # 서비스 프록시 생성 및 호출
            get_waypoints = rospy.ServiceProxy('get_waypoints', WaypointService)
            response = get_waypoints()  # 서비스 호출 -> WaypointServiceResponse 객체 반환

            # 받은 웨이포인트 데이터 저장 및 개수 확인
            self.WP_data = response.waypoint_list.WP_data  
            self.num_k = self.WP_data[0].num.data  # 첫 번째 웨이포인트 번호 설정

            # 로그 출력: 받은 웨이포인트 목록 확인
            rospy.loginfo("Received waypoint list:")
            for wp in response.waypoint_list.WP_data:
                rospy.loginfo(f"Waypoint {wp.num}: x={wp.x}, y={wp.y}")

            return self.WP_data  # 받은 웨이포인트 반환

        except rospy.ServiceException as e:
            # 서비스 호출 실패 시 에러 출력
            rospy.logerr(f"Service call failed: {e}")

    def manage(self, x_ned, y_ned):
        '''
        현재 위치(x_ned, y_ned)를 기반으로 웨이포인트 진행 상태를 관리하는 메서드
        - WP_data에서 현재 및 다음 웨이포인트(WP_k)를 설정 및 갱신
        - 목표 웨이포인트 도착 여부 확인 및 업데이트
        '''
        self.ch = False  # 갱신 여부 초기화

        if not self.WP_data:
            rospy.logwarn("Waypoint data is empty. Please call wp_client first.")
            return  # 웨이포인트 데이터가 없으면 함수 종료

        if not self.WP_k:
            # WP_k에 초기 웨이포인트 추가 (첫 호출 시 초기화)
            self.initialize()

        # 모든 웨이포인트를 다 처리했을 때
        if self.num_k == len(self.WP_data) + 1:
            rospy.loginfo("All waypoints processed.")  
            return

        # 목표 지점과 현재 위치 간 거리 계산
        self.d_goal = self.cal_d_goal(x_ned, y_ned)  

        # 거리 기반 도착 여부 확인 및 갱신
        self.check()

        return self.WP_k  # 현재 및 다음 웨이포인트 반환

    def initialize(self):
        '''
        WP_k에 처음 두 개의 웨이포인트를 할당하는 초기화 메서드
        - 현재 웨이포인트와 다음 웨이포인트를 WP_k 리스트에 추가
        '''
        self.WP_k.append(self.WP_data[self.num_k])      # 현재 웨이포인트 추가
        self.WP_k.append(self.WP_data[self.num_k + 1])  # 다음 웨이포인트 추가
        self.print_update()  # 업데이트된 웨이포인트 정보 출력

    def check(self):
        '''
        현재 위치가 목표 웨이포인트의 range 내에 들어왔는지 확인
        - 도착 시 다음 웨이포인트로 업데이트
        - 도착 로그 출력 및 약간의 지연 추가
        '''
        # if self.d_goal <= self.WP_k[1].range.data:
        if self.d_goal <= self.WP_k[1].range.data*1.5:  # 목표 지점 도착 조건 확인
            rospy.loginfo("Arrive")  
            rospy.sleep(3)  # 도착 시 3초 대기
            self.num_k += 1  # 웨이포인트 인덱스 업데이트

            # 이전 웨이포인트 제거 및 다음 웨이포인트 추가
            self.WP_k.pop(0)  
            # 다음 웨이포인트가 리스트 범위 내에 있는 경우만 추가
            if self.num_k + 1 < len(self.WP_data):
                self.WP_k.append(self.WP_data[self.num_k + 1])

            self.print_update()  # 갱신된 웨이포인트 정보 출력
            self.ch = True  # 갱신 여부 플래그 설정

    def cal_d_goal(self, x_ned, y_ned):
        '''
        현재 위치(x_ned, y_ned)와 목표 웨이포인트 간 거리 계산
        - 두 점 간 거리 공식 사용: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        '''
        return hypot(self.WP_k[1].x.data - x_ned, self.WP_k[1].y.data - y_ned)

    def print_update(self):
        '''
        현재 WP_k 리스트에 있는 웨이포인트 정보를 로그로 출력
        - 현재 진행 중인 웨이포인트 상태 확인용
        '''
        rospy.loginfo("WP_k updated:")
        for waypoint in self.WP_k[:2]:
            rospy.loginfo(f"WP_k Waypoint {waypoint.num}: x={waypoint.x}, y={waypoint.y}")
