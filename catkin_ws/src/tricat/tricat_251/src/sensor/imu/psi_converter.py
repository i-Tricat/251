#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from filterpy.kalman import ExtendedKalmanFilter
from math import atan2, sqrt, cos, sin
import tf.transformations

class IMUEKF:
    def __init__(self):
        # EKF 초기화
        self.ekf = ExtendedKalmanFilter(dim_x=7, dim_z=7)  # 상태는 쿼터니언(x, y, z, w)와 각속도(x, y, z)
        self.dt = 0.01  # 샘플 주기 (100 Hz)
        
        # 초기 상태 [쿼터니언(x, y, z, w), 각속도(x, y, z)]
        self.ekf.x = np.array([0, 0, 0, 1, 0, 0, 0])

        # 상태 전이 행렬
        self.ekf.F = np.eye(7)

        # 제어 입력 행렬 (각속도만 해당)
        self.ekf.B = np.zeros((7, 3))
        self.ekf.B[4:, :] = np.eye(3) * self.dt  # 각속도 부분에만 제어 입력을 적용

        # 프로세스 잡음 공분산
        self.ekf.Q = np.eye(7) * 0.001

        # 측정 잡음 공분산
        self.ekf.R = np.eye(7) * 0.1

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
        q = x[0:4]  # 쿼터니언 (x, y, z, w)
        wx, wy, wz = x[4:7]  # 각속도

        # 쿼터니언을 각속도로 회전
        dq = 0.5 * np.array([[-q[1], -q[2], -q[3]],
                             [ q[0], -q[3],  q[2]],
                             [ q[3],  q[0], -q[1]],
                             [-q[2],  q[1],  q[0]]])
        q_dot = dq.dot([wx, wy, wz])
        q_new = q + q_dot * dt
        q_new /= np.linalg.norm(q_new)  # 정상화

        return np.concatenate([q_new, [wx, wy, wz]])

    def measurement_function(self, x):
        """ 상태를 측정 공간으로 변환 """
        return x  # 상태 벡터 자체를 반환

    def jacobian_of_measurement_function(self, x):
        """ 측정 함수의 야코비안 """
        return np.eye(7)

    def imu_data_callback(self, msg):
        """ IMU 데이터(가속도 및 각속도)에 대한 콜백 """
        self.latest_imu_data = msg
        if self.latest_mag_data is not None:
            self.process_data()

    def imu_mag_callback(self, msg):
        """ 자기장 데이터에 대한 콜백 """
        self.latest_mag_data = msg

    def process_data(self):
        """ 데이터를 처리하고 EKF를 사용하여 결합 """
        imu_msg = self.latest_imu_data
        mag_msg = self.latest_mag_data

        # 각속도
        gx = imu_msg.angular_velocity.x
        gy = imu_msg.angular_velocity.y
        gz = imu_msg.angular_velocity.z

        # 가속도
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        # 자기장
        mx = mag_msg.magnetic_field.x
        my = mag_msg.magnetic_field.y
        mz = mag_msg.magnetic_field.z

        # 가속도 기반으로 롤 및 피치 계산
        roll_acc = atan2(ay, az)
        pitch_acc = atan2(-ax, sqrt(ay**2 + az**2))

        # 쿼터니언 기반으로 지자기 센서를 사용해 요(yaw) 계산
        numerator = my * cos(roll_acc) - mz * sin(roll_acc)
        denominator = (
            mx * cos(pitch_acc) +
            my * sin(roll_acc) * sin(pitch_acc) +
            mz * cos(roll_acc) * sin(pitch_acc)
        )
        yaw_mag = atan2(numerator, denominator)

        # 초기 쿼터니언 계산 (가속도와 자기장 데이터를 이용)
        q = tf.transformations.quaternion_from_euler(roll_acc, pitch_acc, yaw_mag)

        # EKF 예측 단계
        self.ekf.predict(u=np.array([gx, gy, gz]))

        # 측정 벡터 생성 (쿼터니언과 각속도)
        z = np.concatenate([q, [gx, gy, gz]])

        # EKF 업데이트 단계
        self.ekf.update(z, self.jacobian_of_measurement_function, self.measurement_function)

        # 쿼터니언을 오일러 각도로 변환하여 yaw 추출
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.ekf.x[0:4])
        yaw_deg = np.degrees(yaw)

        # yaw 출력
        print("Yaw: {:.3f}°".format(yaw_deg))
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