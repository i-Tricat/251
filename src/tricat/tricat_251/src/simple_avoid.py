#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
from std_msgs.msg import UInt16
from tricat_msgs.msg import Control
from obstacle_detector.msg import Obstacles
from termcolor import colored   # ✅ 컬러 출력용

class SimpleAvoid:
    def __init__(self):
        self.pub = rospy.Publisher("/Control", Control, queue_size=1)
        rospy.Subscriber("/obstacles", Obstacles, self.cb_obstacles)
        self.obstacles = []

        # === 파라미터 ===
        self.base_thrust = rospy.get_param("base_thrust", 1600)
        self.pwm_min     = rospy.get_param("pwm_min", 900)
        self.pwm_max     = rospy.get_param("pwm_max", 2100)
        self.diff_max    = rospy.get_param("diff_max", 350)
        self.forward_axis_sign = rospy.get_param("forward_axis_sign", 1)

        self.obs_range   = rospy.get_param("obs_range", 2.2)
        self.obs_margin  = rospy.get_param("obs_margin", 0.15)
        self.max_angle   = math.radians(rospy.get_param("max_angle_deg", 100.0))
        self.pow_gain    = rospy.get_param("pow_gain", 1.6)
        self.steer_gain  = rospy.get_param("steer_gain", 1.0)
        self.forward_bias= rospy.get_param("forward_bias", 0.6)

        self.rate = rospy.Rate(rospy.get_param("rate", 10))

    def cb_obstacles(self, msg):
        self.obstacles = msg.circles

    def step(self):
        # === 장애물 처리 ===
        obstacles = self.obstacles
        obstacles_used = 0
        steer = 0.0

        for ob in obstacles:
            dx = -ob.center.x * self.forward_axis_sign
            dy =  ob.center.y
            d  = math.hypot(dx, dy)
            if d > self.obs_range + ob.radius + self.obs_margin:
                continue

            angle = math.atan2(dy, dx)
            if abs(angle) > self.max_angle/2:
                continue

            # 가중치 계산
            w = (1.0/(d+1e-6))**self.pow_gain
            if abs(angle) < math.radians(20):
                w *= (1.0 + self.forward_bias)

            steer += -math.copysign(1.0, angle) * w
            obstacles_used += 1

        # 정규화
        steer = max(min(steer * self.steer_gain, 1.0), -1.0)

        # PWM 계산
        diff  = int(steer * self.diff_max)
        left  = self.base_thrust + diff
        right = self.base_thrust - diff
        left  = max(min(left,  self.pwm_max), self.pwm_min)
        right = max(min(right, self.pwm_max), self.pwm_min)

        # === 퍼블리시 ===
        msg = Control()
        msg.thruster_p = UInt16(int(left))
        msg.thruster_s = UInt16(int(right))
        self.pub.publish(msg)

        # === 읽기 편한 상태 출력 ===
        print("=" * 50)
        print(colored("[SimpleAvoid 상태]", "cyan", attrs=["bold"]))
        print(f"{colored('총 인식된 장애물', 'yellow')}: {len(obstacles)}")
        print(f"{colored('사용된 장애물', 'yellow')}: {obstacles_used}")

        # 방향 해석
        if abs(steer) < 0.1:
            direction = "직진"
        elif steer > 0:
            direction = "좌측 회피"
        else:
            direction = "우측 회피"

        # 세부 출력
        print(f"{colored('조향 값(steer)', 'green')}: {steer:+.2f}")
        print(f"{colored('이동 방향', 'magenta')}: {direction}")
        print(f"{colored('좌측 PWM', 'red')}: {left:4d}   "
              f"{colored('우측 PWM', 'red')}: {right:4d}")
        print("=" * 50)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("simple_avoid")
    node = SimpleAvoid()
    node.run()
