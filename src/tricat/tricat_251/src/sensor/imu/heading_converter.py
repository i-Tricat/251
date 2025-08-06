#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from filterpy.kalman import ExtendedKalmanFilter
from math import atan2, sqrt, cos, sin

class IMUEKF:
    def __init__(self):
        # EKF 초기화
        self.ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
        self.dt = 0.01  # 샘플 주기 (100 Hz)
        
        # 초기 상태 [롤, 피치, 요, 롤 속도, 피치 속도, 요 속도]
        self.ekf.x = np.zeros(6)

        # 상태 전이 행렬
        self.ekf.F = np.eye(6)
        self.ekf.F[0, 3] = self.dt
        self.ekf.F[1, 4] = self.dt
        self.ekf.F[2, 5] = self.dt

        # 제어 입력 행렬 (각속도를 위한)
        self.ekf.B = np.eye(6, 3) * self.dt

        # 프로세스 잡음 공분산
        self.ekf.Q = np.eye(6) * 0.001

        # 측정 잡음 공분산
        self.ekf.R = np.eye(3) * 0.1

        # 초기 공분산 행렬
        self.ekf.P *= 10

        # ROS 구독자 설정
        self.imu_data_sub = rospy.Subscriber('/imu/data', Imu, self.imu_data_callback)
        self.imu_mag_sub = rospy.Subscriber('/imu/mag', MagneticField, self.imu_mag_callback)
        
        # 요(yaw) 각도를 위한 ROS 퍼블리셔
        self.yaw_pub = rospy.Publisher('/psi', Float64, queue_size=10)

        # 최신 센서 데이터를 저장할 변수
        self.latest_imu_data = None
        self.latest_mag_data = None

    def state_transition_function(self, x, dt):
        """ 현재 상태와 경과 시간 dt를 기반으로 다음 상태 예측 """
        roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate = x
        
        roll += roll_rate * dt
        pitch += pitch_rate * dt
        yaw += yaw_rate * dt

        return np.array([roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate])
    
    

    def measurement_function(self, x):
        """ 상태를 측정 공간으로 변환 """
        roll, pitch, yaw = x[0:3]
        return np.array([roll, pitch, yaw])

    def jacobian_of_measurement_function(self, x):
        """ 측정 함수의 야코비안 """
        # 측정 함수가 각도를 직접 측정한다고 가정
        return np.eye(3, 6)  # 롤, 피치, 요만 관측하고 속도는 무시

    def imu_data_callback(self, msg):
        """ IMU 데이터(선형 가속도 및 각속도)에 대한 콜백 """
        self.latest_imu_data = msg

        # IMU와 자기 데이터가 모두 있을 때 데이터를 처리
        if self.latest_mag_data is not None:
            self.process_data()

    def imu_mag_callback(self, msg):
        """ 자기장 데이터에 대한 콜백 """
        self.latest_mag_data = msg

    def process_data(self):
        """ 데이터를 처리하고 EKF를 사용하여 결합 """
        imu_msg = self.latest_imu_data
        mag_msg = self.latest_mag_data

        # 각속도(자이로스코프) 데이터(rad/s)
        gx = imu_msg.angular_velocity.x
        gy = imu_msg.angular_velocity.y
        gz = imu_msg.angular_velocity.z

        # EKF 예측: 자이로스코프 데이터를 사용하여 각속도 업데이트
        self.ekf.predict(u=np.array([gx, gy, gz]))

        # 가속도계 데이터
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        # 가속도계로부터 롤 및 피치 계산
        roll_acc = atan2(ay, az)
        pitch_acc = atan2(-ax, sqrt(ay**2 + az**2))

        # 자기장 데이터
        mx = mag_msg.magnetic_field.x
        my = mag_msg.magnetic_field.y
        mz = mag_msg.magnetic_field.z

        # 가속도계 및 자기장 데이터로부터 요 계산
        yaw_mag = atan2(
            my * cos(roll_acc) - mz * sin(roll_acc),
            mx * cos(pitch_acc) + my * sin(roll_acc) * sin(pitch_acc) + mz * cos(roll_acc) * sin(pitch_acc)
        )

        # EKF 업데이트
        z = np.array([roll_acc, pitch_acc, yaw_mag])
        self.ekf.update(z, self.jacobian_of_measurement_function, self.measurement_function)

        # 추정된 상태 추출
        roll, pitch, yaw = self.ekf.x[0:3]

        # 각도를 도(degree)로 변환
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # rospy.loginfo("Estimated Orientation - Roll: {:.3f}°, Pitch: {:.3f}°, Yaw: {:.3f}°".format(roll_deg, pitch_deg, yaw_deg))

        # 추정된 요를 도(degree)로 퍼블리시
        self.yaw_pub.publish(yaw_deg)

def main():
    rospy.init_node('imu_ekf_node', anonymous=True)
    imu_ekf = IMUEKF()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




