#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from tricat_msgs.msg import Sensor_total, Pose, ObstacleList

class SensorProcessor:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('Ship_state', anonymous=True)

        # 퍼블리셔
        self.pose_pub = rospy.Publisher('/Pose', Pose, queue_size=10)
        # self.obstacle_list_pub = rospy.Publisher('/Obstacles', ObstacleList, queue_size=10)

        # 서브스크라이버
        rospy.Subscriber('/sensor_total', Sensor_total, self.sensor_total_callback)

        # 속도 계산 초기화
        self.prev_position = None
        self.prev_time = None
        self.prev_psi = None

    def calculate_velocities(self, current_position, current_time, current_psi):
        # 첫 번째 계산의 경우 초기화
        if self.prev_position is None or self.prev_time is None or self.prev_psi is None:
            self.prev_position = current_position
            self.prev_time = current_time
            self.prev_psi = current_psi
            return 0.0, 0.0, 0.0

        # 시간 차 계산
        time_diff = (current_time - self.prev_time).to_sec()

        # 시간 차가 0인 경우를 방지 (예외 처리)
        if time_diff <= 0:
            return 0.0, 0.0, 0.0

        # 속도 성분 계산
        u = (current_position.x - self.prev_position.x) / time_diff
        v = (current_position.y - self.prev_position.y) / time_diff

        # 각속도 r 계산 (deg/s)
        r = (current_psi - self.prev_psi) / time_diff 

        # 이전 위치, 시간 및 psi 업데이트
        self.prev_position = current_position
        self.prev_time = current_time
        self.prev_psi = current_psi

        return u, v, r

    def sensor_total_callback(self, msg):
        # 현재 시간 가져오기
        current_time = rospy.Time.now()

        # 속도 계산
        u, v, r = self.calculate_velocities(msg.position_ned, current_time, msg.psi.data)

        # Pose 메시지 생성
        pose_msg = Pose()
        pose_msg.x = Float64(msg.position_ned.x)
        pose_msg.y = Float64(msg.position_ned.y)
        pose_msg.psi = Float64(msg.psi.data)
        pose_msg.u = Float64(u)
        pose_msg.v = Float64(v)
        pose_msg.r = Float64(r)

        # Pose 메시지 퍼블리시
        self.pose_pub.publish(pose_msg)

        # ObstacleList 메시지 퍼블리시
        # self.obstacle_list_pub.publish(msg.obstacles)

        # 터미널에 출력
        rospy.loginfo(
            f"x: {pose_msg.x.data:.3f} m\n"
            f"y: {pose_msg.y.data:.3f} m\n"
            f"u: {pose_msg.u.data:.3f} m/s\n"
            f"v: {pose_msg.v.data:.3f} m/s\n"
            f"psi: {pose_msg.psi.data:.3f} deg\n"
            f"r: {pose_msg.r.data:.3f} deg/s"
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = SensorProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass