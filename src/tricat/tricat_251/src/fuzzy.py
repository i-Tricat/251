#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math
from math import sin, cos, radians, degrees, hypot, atan2, pi, sqrt, isnan, isinf
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from tricat_msgs.msg import Pose, Control
from control.autopilot import heading_cal
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, UInt16, Bool
from sensor_msgs.msg import LaserScan
from ship.wp_manager import WpManager

D2R = pi / 180
R2D = 180 / pi

class Fuzzy:
    def __init__(self):
         ### GPS(Waypoint) ###
        self.wp_manager=WpManager()
        self.wp_manager.wp_client()
        self.WP_data = []
        self.WP_k = []
        self.num_k = 0
        self.d_goal = 0
        self.target_heading = 0.0  # ✅ target_heading 기본값 추가
        self.psi_diff= 0.0    # (추가) target_angle도 함께 초기화
        self.ch = False

        if self.wp_manager.WP_data:
            self.wp_manager.initialize()

        ### psi_d ### 
        self.psi_d_ned = 0.0

        ### /Pose ###
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.u_ned = 0.0
        self.v_ned = 0.0
        self.r_ned = 0.0
        self.U = 0.0

        self.thrust_range = rospy.get_param("thrust_range", [1100, 1900])
        self.base_thrust = rospy.get_param("base_thrust", 1500)

        #PID Control
        self.errSum = 0
        self.kp_thruster = rospy.get_param("kp_thruster")
        self.kd_thruster = rospy.get_param("kd_thruster")
        
        #Lidar
        self.obstacles = []
        self.angle_min = 0.0 
        self.angle_increment = 0.0
        self.ranges = []
        self.danger_ob = {}

        #ROS
        # sub
        self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # pub
        self.control_msg = Control()
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        self.finish_pub = rospy.Publisher("/finish_check", Bool, queue_size=1) # YOO 카메라 관련된 네이밍 필요 
        self.finish = False

        #Fuzzy
        self.fuzzy_thrust_control =  0
        self.target_thrust_ang = None
        self.clo=0
    
    def pose_callback(self, msg):
        try:
            self.x_ned = float(msg.x.data)
            self.y_ned = float(msg.y.data)
            psi_deg = float(msg.psi.data)
            self.psi_ned = ((psi_deg * D2R + pi) % (2 * pi)) - pi  # deg -> rad

            rospy.loginfo_throttle(2, f"Pose 수신: x={self.x_ned}, y={self.y_ned}, psi(deg)={psi_deg}, psi(rad)={self.psi_ned}")

            self.pose_received = True

        except Exception as e:
            rospy.logwarn(f"Pose 값 수신 오류: {e}")

    def lidar_callback(self, data):
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        self.ranges = data.ranges

    def info(self):
        ### base ###
        self.d_goal = self.wp_manager.cal_d_goal(self.x_ned, self.y_ned)
        self.WP_k = self.wp_manager.manage(self.x_ned, self.y_ned)
        self.target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned)
        self.target_angle = self.target_heading - self.psi_ned
        self.rerange_angle()  # 🔧 output_angle 생성

    def fuzzy(self):
        # Define input variables
        distance = ctrl.Antecedent(np.arange(0, 4, 0.1), "distance")
        angle = ctrl.Antecedent(np.arange(-70, 70, 1), "angle")
        
        # Define output variable
        target_thrust = ctrl.Consequent(np.arange(-30, 30, 1), "target_thrust")
        
        # Define membership functions for input variables
        ## distance
        distance["ED"] = fuzz.trapmf(distance.universe, [0, 0, 0.5, 1.5])
        distance["D"] = fuzz.trimf(distance.universe, [0.5, 1.5, 2.5])
        distance["W"] = fuzz.trimf(distance.universe, [1.5, 2.5, 3.5])
        distance["B"] = fuzz.trapmf(distance.universe, [2.5, 3.5, 4, 4])
        ## angle
        angle["NL"] = fuzz.trapmf(angle.universe, [-70, -50, -40, -30]) # Negative Large
        angle["NM"] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10]) # Negative Medium
        angle["NS"] = fuzz.trimf(angle.universe, [-25, 0, 1])           # Negative Small
        angle["PS"] = fuzz.trimf(angle.universe, [0, 1, 25])            # Positive Small
        angle["PM"] = fuzz.trapmf(angle.universe, [10, 20, 30, 40])     # Positive Medium
        angle["PL"] = fuzz.trapmf(angle.universe, [30, 40, 50, 70])     # Positive Large

        # Define membership functions for output variable
        ## Thruster
        target_thrust["RE"] = fuzz.trimf(target_thrust.universe, [-30, -27, -18]) # Rigtht Extra
        target_thrust["RL"] = fuzz.trimf(target_thrust.universe, [-18, -16, -13]) # Right Large
        target_thrust["RM"] = fuzz.trimf(target_thrust.universe, [-18, -13, -7])  # Right Medium
        target_thrust["RS"] = fuzz.trimf(target_thrust.universe, [-12, -7, 0])    # Right Small
        target_thrust["N"] = fuzz.trimf(target_thrust.universe, [0, 0, 0])
        target_thrust["LS"] = fuzz.trimf(target_thrust.universe, [0, 7, 12])      # Left Small
        target_thrust["LM"] = fuzz.trimf(target_thrust.universe, [7, 13, 18])     # Left Medium
        target_thrust["LL"] = fuzz.trimf(target_thrust.universe, [13, 16, 18])    # Left Large
        target_thrust["LE"] = fuzz.trimf(target_thrust.universe, [18, 27, 30])    # Left Extra

        # Define rules
        rule_ED_NL = ctrl.Rule(distance["ED"] & angle["NL"], target_thrust["RM"])
        rule_ED_NM = ctrl.Rule(distance["ED"] & angle["NM"], target_thrust["RL"])
        rule_ED_NS = ctrl.Rule(distance["ED"] & angle["NS"], target_thrust["RE"])
        rule_ED_PS = ctrl.Rule(distance["ED"] & angle["PS"], target_thrust["LE"])
        rule_ED_PM = ctrl.Rule(distance["ED"] & angle["PM"], target_thrust["LL"])
        rule_ED_PL = ctrl.Rule(distance["ED"] & angle["PL"], target_thrust["LM"])

        rule_D_NL = ctrl.Rule(distance["D"] & angle["NL"], target_thrust["RS"])
        rule_D_NM = ctrl.Rule(distance["D"] & angle["NM"], target_thrust["RM"])
        rule_D_NS = ctrl.Rule(distance["D"] & angle["NS"], target_thrust["RL"])
        rule_D_PS = ctrl.Rule(distance["D"] & angle["PS"], target_thrust["LL"])
        rule_D_PM = ctrl.Rule(distance["D"] & angle["PM"], target_thrust["LM"])
        rule_D_PL = ctrl.Rule(distance["D"] & angle["PL"], target_thrust["LS"])

        rule_W_NL = ctrl.Rule(distance["W"] & angle["NL"], target_thrust["N"])
        rule_W_NM = ctrl.Rule(distance["W"] & angle["NM"], target_thrust["RS"])
        rule_W_NS = ctrl.Rule(distance["W"] & angle["NS"], target_thrust["RM"])
        rule_W_PS = ctrl.Rule(distance["W"] & angle["PS"], target_thrust["LM"])
        rule_W_PM = ctrl.Rule(distance["W"] & angle["PM"], target_thrust["LS"])
        rule_W_PL = ctrl.Rule(distance["W"] & angle["PL"], target_thrust["N"])

        rule_B_NL = ctrl.Rule(distance["B"] & angle["NL"], target_thrust["N"])
        rule_B_NM = ctrl.Rule(distance["B"] & angle["NM"], target_thrust["N"])
        rule_B_NS = ctrl.Rule(distance["B"] & angle["NS"], target_thrust["RS"])
        rule_B_PS = ctrl.Rule(distance["B"] & angle["PS"], target_thrust["LS"])
        rule_B_PM = ctrl.Rule(distance["B"] & angle["PM"], target_thrust["N"])
        rule_B_PL = ctrl.Rule(distance["B"] & angle["PL"], target_thrust["N"])

        # Base class to contain a Fuzzy Control System
        target_thrust_ctrl = ctrl.ControlSystem(
            [rule_ED_NL, rule_ED_NM, rule_ED_NS, rule_ED_PL, rule_ED_PM, rule_ED_PS,
             rule_D_NL, rule_D_NM, rule_D_NS, rule_D_PL, rule_D_PM, rule_D_PS,
             rule_W_NL, rule_W_NM, rule_W_NS, rule_W_PL, rule_W_PM, rule_W_PS,
             rule_B_NL, rule_B_NM, rule_B_NS, rule_B_PL, rule_B_PM, rule_B_PS])
        
        self.target_thrust_ang = ctrl.ControlSystemSimulation(target_thrust_ctrl)
    
    def fuzzy_control_avoidance(self):
        self.danger_ob = {}

        start_idx = int((math.radians(110)) / (self.angle_increment + 0.00001))
        end_idx = int((math.radians(250)) / (self.angle_increment + 0.00001))
        ranges = self.ranges[start_idx : (end_idx + 1)]

        if ranges == [] or min(ranges) == float("inf"):
            return False

        closest_distance = min(ranges)

        self.clo=closest_distance

        idx = ranges.index(closest_distance) + start_idx
        pi = math.degrees(self.angle_min + self.angle_increment * idx)

        if (0.3 <= closest_distance <= 2.8) and (-70 <= pi <= 70):
            self.target_thrust_ang.input["distance"] = float(closest_distance)
            self.target_thrust_ang.input["angle"] = float(pi)
            self.target_thrust_ang.compute()
            self.fuzzy_thrust_control = int(self.target_thrust_ang.output["target_thrust"])

            self.danger_ob["distance"] = closest_distance
            self.danger_ob["idx"] = idx
            self.danger_ob["pi"] = pi

            return True
        else:
            return False
        
    def rerange_angle(self):
        if degrees(self.target_angle) >= 180:
            self.output_angle = -180 + abs(degrees(self.target_angle)) % 180
        elif degrees(self.target_angle) <= -180:
            self.output_angle = 180 - abs(degrees(self.target_angle)) % 180
        else:
            self.output_angle = degrees(self.target_angle)
        return self.output_angle
    
    def thrust_pid_controller(self, psi_ned, x_ned, y_ned, target_angle):
        control_angle = self.target_angle

        if control_angle >= 180:
            control_angle = -180 + abs(control_angle) % 180
        elif control_angle <= -180:
            control_angle = 180 - abs(control_angle) % 180
        
        self.control_angle = control_angle 
        cp_thrust = self.kp_thruster * control_angle
        yaw_rate = degrees(self.r_ned)
        cd_thrust = self.kd_thruster * (-yaw_rate)

        thrust_diff = cp_thrust + cd_thrust  # 좌우 추진기 차등 추력 계산

        # 기본 추력 설정
        base_thrust = self.base_thrust  # 기본 전진 추력
        left_thrust = base_thrust + thrust_diff
        right_thrust = base_thrust - thrust_diff

        # 추력 범위 제한
        self.thruster_p = max(min(left_thrust, self.thrust_range[1]), self.thrust_range[0])
        self.thruster_s = max(min(right_thrust, self.thrust_range[1]), self.thrust_range[0])

        return self.thruster_p, self.thruster_s

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)
        
    def print_state(self):
        print(f"------------------------------------\n \
            lpp : {self.fuzzy_control_avoidance()}\n \
            distance : {self.d_goal}\n \
            my xy : {self.x_ned}, {self.y_ned}\n \
            goal xy : {self.WP_k[1].x.data}, {self.WP_k[1].y.data}\n \
            psi, desire : {degrees(self.psi_ned)}, {self.output_angle}\n \
            thruster : L = {self.thruster_p}, R = {self.thruster_s}\n \
            range : {self.clo}\n \
            target rule : {self.fuzzy_thrust_control}")
        
def main():
    rospy.init_node("Fuzzy", anonymous=False)
    rate = rospy.Rate(10) # 10 Hz
    fuzzy = Fuzzy()
    fuzzy.fuzzy() 

    while not rospy.is_shutdown():
        fuzzy.info()  # 위치/타겟 등 정보 갱신

        is_lpp = fuzzy.fuzzy_control_avoidance()
        if is_lpp:
            # 퍼지 제어 기반 회피: fuzzy_thrust_control 사용
            base = fuzzy.base_thrust
            diff = fuzzy.fuzzy_thrust_control

            fuzzy.thruster_p = max(min(base + diff, fuzzy.thrust_range[1]), fuzzy.thrust_range[0])
            fuzzy.thruster_s = max(min(base - diff, fuzzy.thrust_range[1]), fuzzy.thrust_range[0])

        else:
            # 일반 PID 제어 방식
            fuzzy.thrust_pid_controller(
                fuzzy.psi_ned, fuzzy.x_ned, fuzzy.y_ned, fuzzy.target_angle
            )
        fuzzy.control_publish()
        fuzzy.print_state()
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()