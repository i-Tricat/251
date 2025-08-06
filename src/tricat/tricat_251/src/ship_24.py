#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from tricat241_pkg.msg import Pose, WPList, Control, ObstacleList
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt16
from control.controller import SteeringController
from control.autopilot import HeadingAutoPilot, heading_cal
from ship.wp_manager import WpManager
from guidance.static_ob_cal import *
import numpy as np
from math import degrees, radians, pi, sqrt
from ship.model.otter import dynamics, HeadingAutoPilot2
from docking.predock import PreDockController, PredockCal, STController

D2R = pi / 180
R2D = 180 / pi

class ship:
    '''
    Using NED Cordinate
    '''
    def __init__(self, Model = False):
        self.br = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

        # State
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.psi_ned = 0.0
        self.u_ned = 0.0
        self.v_ned = 0.0
        self.r_ned = 0.0

        self.U = 0.0
        self.psi_d_ned = 0.0

        if not Model:
            # Ship data
            self.ship_name = '241'
            self.DOF = 3
            self.control_mode = 'headingAutopilot'
            self.controller_type = "headingController"

            if self.controller_type == "headingController":
                self.init_heading_controller()

            elif self.controller_type == "predockcontroller":
                self.init_predock_controller()     

            # ROS Subsribers
            self.pose_sub = rospy.Subscriber("/Pose", Pose, self.pose_callback, queue_size=1)

            # ROS Publishers
            self.control_msg = Control()
            self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)
        
        else:
            self.init_simulation_model()

        # Waypoin data
        self.WP_data = []
        self.WP_k = []

        # Waypoint Manager
        self.d_goal = 0   # 목표지점과의 거리                               
        self.wp_manager = WpManager()


    def broadcast_transform(self):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "map_ned"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(pi, 0, -pi/2)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        transform.header.frame_id = "map_ned"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x_ned
        transform.transform.translation.y = self.y_ned
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.psi_ned)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # base_link to left_hull
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "left_hull"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.4
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # base_link to right_hull
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "right_hull"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = -0.4
        transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # left_hull to servo1
        transform.header.frame_id = "left_hull"
        transform.child_frame_id = "servo1"
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.125
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # right_hull to servo2
        transform.header.frame_id = "right_hull"
        transform.child_frame_id = "servo2"
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.125
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # servo1 to thruster1
        transform.header.frame_id = "servo1"
        transform.child_frame_id = "thruster1"
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # servo2 to thruster2
        transform.header.frame_id = "servo2"
        transform.child_frame_id = "thruster2"
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # base_link to imu
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "imu"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # base_link to lidar
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "velodyne"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.3
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # base_link to camera
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera"
        transform.transform.translation.x = 0.1
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.25
        q = quaternion_from_euler(0, 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.br.sendTransform(transform)

        # odometry 메시지 작성
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map_ned"
        
        # Position
        odom.pose.pose.position.x = self.x_ned
        odom.pose.pose.position.y = self.y_ned
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.psi_ned)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.u_ned
        odom.twist.twist.linear.y = self.v_ned
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = self.r_ned
        
        # Odometry 메시지 발행
        self.odom_pub.publish(odom)

    def change_controller(self, controller_type):
            self.controller_type = controller_type
            if self.controller_type == "predockcontroller":
                self.init_predock_controller()

            elif self.controller_type == "dockcontroller":
                self.init_dock_controller()

            elif self.controller_type == "headingAutopilot":
                self.init_heading_controller()

####################################################################################################
#                                     heading_control                                              #      
####################################################################################################

    def init_heading_controller(self):

        # ROS Subsribers
        rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=10)
        self.obstacles_list = np.empty((0, 4))
        self.angle_num = rospy.get_param("angle_num", 10)
        self.yaw_range = rospy.get_param("yaw_range", 30)
        self.range = rospy.get_param("range", 10)
        self.margin = rospy.get_param("margin", 0.5)

        self.static_ob_cal = StaticOBCal(self.obstacles_list, self.angle_num, self.yaw_range, self.range, self.margin)

        # heading autopilot
        kp_servo = rospy.get_param("kp_servo")
        kd_servo = rospy.get_param("kd_servo")
        self.autopilot = HeadingAutoPilot(kp_servo, kd_servo, self.psi_d_ned)
        self.delta_c = 0

        # steering controller
        self.servo_k = 4
        servo_range = rospy.get_param("servo_range")  # [80, 110]
        self.servo_middle = int((servo_range[0] + servo_range[1]) / 2)  # [95]
        delta_max = [radians((servo_range[0] - self.servo_middle - 5) * self.servo_k),
                     radians((servo_range[1] - self.servo_middle) * self.servo_k)]  # [rad]
        rospy.loginfo(delta_max)
        delta_dot_max = rospy.get_param("delta_dot_max")  # 2.09 [rad/s]
        self.steering_controller_s = SteeringController(delta_max, delta_dot_max, self.delta_c)
        self.steering_controller_p = SteeringController(delta_max, delta_dot_max, self.delta_c)
        self.servo_s = self.steering_controller_s.delta_c
        self.servo_p = self.steering_controller_p.delta_c

        # thruster
        self.thruster_s = 0
        self.thruster_p = 0

    def obstacle_callback(self, msg):
        self.obstacles_list = np.array([[i.begin.x, -i.begin.y, i.end.x, -i.end.y] for i in msg.obstacle])
        self.static_ob_cal.generate_vectors()

    def closest_vector(self, available_angles, target_heading):
        min_diff = float('inf')
        closest_angle = target_heading
        for angle in available_angles:
            diff = abs((target_heading - angle + np.pi) % (2 * np.pi) - np.pi)
            # (target_heading - angle + pi)을 2*pi로 나눈 나머지에서 pi를 빼면, 차이가 항상 -pi에서 pi 사이에 있게 됨
            if diff < min_diff:
                min_diff = diff
                closest_angle = angle
        return closest_angle

    def normalize_angle(self, angle):
        """Normalize an angle to be within the range [-pi, pi]."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def autopilot_run(self):
        self.static_ob_cal.set_obstacles(self.obstacles_list)
        false_angles = self.static_ob_cal.cross_check()
        print(false_angles)
        available_angle = self.static_ob_cal.angle_convert_to_ned(false_angles)*D2R + self.psi_ned # rad
        target_heading = heading_cal(self.WP_k[1].x.data, self.WP_k[1].y.data, self.x_ned, self.y_ned) # rad
        # print(target_heading*R2D)
        self.psi_d_ned = self.closest_vector(available_angle, target_heading)
        self.autopilot.set_psi_d(self.normalize_angle(self.psi_d_ned))
        self.delta_c = self.autopilot.update(self.psi_ned, self.r_ned) # [rad]
        self.servo_s = self.servo_middle - int(degrees(self.steering_controller_s.compute(self.delta_c))/self.servo_k)
        self.servo_p = self.servo_middle - int(degrees(self.steering_controller_p.compute(self.delta_c))/self.servo_k)

####################################################################################################
#                                       Pre Docking                                                # 
####################################################################################################
    def init_predock_controller(self):
        self.control_mode = 'PreDocking'

        self.point_cloud_data = None
        rospy.Subscriber("/velodyne_points", PointCloud2, self.point_cloud_callback, queue_size=10)

        self.predockCal = PredockCal()
        self.predockcontroller = PreDockController()

    def point_cloud_callback(self, data):
        if data is None:
            rospy.logwarn("Received empty PointCloud2 data")
            return
        
        if not isinstance(data, PointCloud2):
            rospy.logerr("Expected PointCloud2 data but received different type")
            return

        self.point_cloud_data = data
    
    def predock_run(self):
        self.predockCal.get_point_cloud_data(self.point_cloud_data)
        self.predockCal.process_point_cloud()
        self.predockcontroller.get_info(self.predockCal.front_distance, 
                                        self.predockCal.right_distance, 
                                        self.predockCal.left_distance,
                                        self.predockCal.front_wall_angle,
                                        self.predockCal.right_wall_angle,
                                        self.predockCal.left_wall_angle)
        self.predockcontroller.run()
        self.servo_p = self.predockcontroller.servo_p
        self.servo_s = self.predockcontroller.servo_s
        self.thruster_p = self.predockcontroller.thruster_p
        self.thruster_s = self.predockcontroller.thruster_s

####################################################################################################
#                                          Docking                                                 # 
####################################################################################################
    def init_dock_controller(self):
        self.control_mode = 'Docking'
        self.dockcontroller = STController()

####################################################################################################
#                                       Simulation                                                 #      
####################################################################################################
    def init_simulation_model(self):
        self.ship_name = 'Otter USV'
        self.DOF = 6
        self.control_mode = 'headingAutopilot'
        self.dt = 0.1

        self.eta = np.array([0, 0, 0, 0, 0, 0], float)  # position/attitude
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector

        self.dynamics = dynamics(self.eta, self.nu)
        self.autopilot = HeadingAutoPilot2(self.psi_d_ned, self.dt)
        
        self.u_control = self.autopilot.update(self.eta, self.nu, self.dynamics.Binv)
        self.u_actual = self.dynamics.update(self.u_control, self.dt)

####################################################################################################
#                                        Autonomous                                               #      
####################################################################################################
    def pose_callback(self, msg):
        self.x_ned = float(msg.x.data)
        self.y_ned = float(msg.y.data)
        self.psi_ned = (float(msg.psi.data)*D2R + pi) % (2 * pi) - pi # deg -> rad
        self.u_ned = float(msg.u.data)
        self.v_ned = float(msg.v.data)
        self.r_ned = float(msg.r.data)*D2R # deg/s -> rad/s
        self.U = sqrt(self.u_ned**2 + self.v_ned**2)

    def control_publish(self):
        self.broadcast_transform()
        self.control_msg.servo_p = UInt16(self.servo_p)
        self.control_msg.servo_s = UInt16(self.servo_s)
        self.control_msg.thruster_p = UInt16(self.thruster_p)
        self.control_msg.thruster_s = UInt16(self.thruster_s)
        self.control_pub.publish(self.control_msg)

    def print_state(self):
        print(
            f"------------------------------------\n"
            f"control mode: {self.control_mode}\n"
            f"count, goal_x, goal_y : {self.WP_k[1].num.data}, {self.WP_k[1].x.data}, {self.WP_k[1].y.data}\n"
            f"x, y : {self.x_ned}, {self.y_ned}\n"
            f"psi, psi_d : {round(degrees(self.psi_ned),2)}, {round(degrees(self.psi_d_ned),2)}\n"
            f"distance : {self.d_goal}\n"
            f"servo_s, servo_p: {self.servo_s}, {self.servo_p}\n"
            f"thruster_s, thruster_p : {self.thruster_s}, {self.thruster_p}\n")