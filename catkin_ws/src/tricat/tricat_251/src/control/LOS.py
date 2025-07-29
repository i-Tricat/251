#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os
import numpy as np
from math import cos, sin, atan, atan2, sqrt, pi, exp

def cal_cross_track_error(x_t_ned, y_t_ned, x_ref_ned, y_ref_ned, x_ned, y_ned, x_p_ned, y_p_ned, pi_p):
        '''
        Calculate the cross track error, y_e
        Input: 
            (x_t_ned, y_t_ned):       target position
            (x_ref_ned, y_ref_ned):   reference position
            (x_ned, y_ned):           boat positions (m)
        
        Return: 
            (x_p_ned, y_p_ned): origin of path-tangential frame
            y_e:                cross-track error (m)
        '''
        pi_p = np.arctan2(y_t_ned - y_ref_ned, x_t_ned - x_ref_ned)

        cp = np.cos(pi_p)
        sp = np.sin(pi_p)
        tp = np.tan(pi_p)

        A = np.array([[cp,  sp, 0],
                      [-sp, cp, 1],
                      [tp,  -1, 0]])
        
        b = np.array([cp*x_ned + sp*y_ned,
                      -sp*x_ned + cp*y_ned,
                      tp*x_t_ned - y_t_ned])

        x = np.linalg.solve(A, b)

        x_p_ned = x[0]
        y_p_ned = x[1]
        y_e = x[2]
        
        return x_p_ned, y_p_ned, y_e

class Los:
    def __init__(self, waypoints, delta):
        '''
        Input:
            waypoints: 
            delta: look-ahead distance
        '''
        self.waypoints = waypoints
        self.k = self.waypoints['k'][0]
        self.xk_ned = self.waypoints['x'][0]
        self.yk_ned = self.waypoints['y'][0]
        self.xk_next_ned = self.waypoints['x'][1]
        self.yk_next_ned = self.waypoints['y'][1]
        self.R_switch = self.waypoints['r'][1]

        if delta == None:
            self.d = sqrt((self.xk_next_ned - self.xk_ned)**2 + (self.yk_next_ned - self.yk_ned)**2)
            self.delta = self.d/2 #TODO 특정 값으로 하는게 맞는지 확인 후 수정 필요
        elif delta == 0:
            print("wrong input! delta > 0")
            sys.exit(1) #TODO 다른 방법은 없는지 확인 필요
        else:
            self.delta = delta
        self.K_p = 1/self.delta

        self.pi_h_ned = atan2(self.yk_next_ned - self.yk_ned, self.xk_next_ned - self.xk_ned) # path-tangetial
    
    def check(self, x_e_ned):
        '''
        Check if we need to switch to the next waypoint
        '''
        if abs(x_e_ned) < self.R_switch:
            self.k += 1
        
        return self.k

    def update_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.k = self.waypoints['k'][0]
        self.xk_ned = self.waypoints['x'][0]
        self.yk_ned = self.waypoints['y'][0]
        self.xk_next_ned = self.waypoints['x'][1]
        self.yk_next_ned = self.waypoints['y'][1]
        
        print(f"Waypoint update! Active waypoints:\n  (x1, y1) = ({self.xk_ned:.2f}, {self.yk_ned:.2f})")

    #TODO beta 구하는 부분 수정 필요 
    def compute(self, x_ned, y_ned, v, u):
        '''
        beta: crab angle
        '''
        x_e_ned = (x_ned - self.xk_ned) * cos(self.pi_h_ned) + (y_ned - self.yk_ned) * sin(self.pi_h_ned)
        y_e_ned = -(x_ned - self.xk_ned) * sin(self.pi_h_ned) + (y_ned - self.yk_ned) * cos(self.pi_h_ned)
        beta_ned = atan2(v, u)
        psi_d = self.pi_h_ned + atan(-self.K_p*y_e_ned) - beta_ned

        return x_e_ned, y_e_ned, psi_d

class Alos(Los):
    def __init__(self, waypoints, delta, gamma, h):
        '''
        Adaptive LOS - Fossen(2023):
            Compute the desired headig angle(psi_d), desired yaw rate(r_d), cross-track error(y_e)
            when path is straight lines going through the waypoints

        Inputs:   
            waypoints: 
            R_switch:  go to next waypoint when the along-track distance x_e is less than R_switch (m)
            delta:     positive look-ahead distance (m), (typically 5-20 m)
            gamma:     positive adaptive gain constant, (typically 0.001)
            h:         sampling time (s)
        '''
        super().__init__(waypoints, delta)
        if gamma < 0:
            print("wrong input, gamma > 0")
        else:
            self.gamma = gamma
        self.h = h
        self.beta_hat = 0

    def compute(self, x_ned, y_ned):
        '''
        Calculate Along-track errors (x_e), Cross-track errors (y_e) and heading desired angle

        Inputs:   
            (x_ned, y_ned): boat positions (m)
        
        Outputs:
            x_e_ned: along-track error (m)
            y_e_ned: cross-track error (m)
            psi_d_ned: LOS heading angle (rad)
            
        '''
        x_e_ned = (x_ned - self.xk_ned) * cos(self.pi_h_ned) + (y_ned - self.yk_ned) * sin(self.pi_h_ned)
        y_e_ned = -(x_ned - self.xk_ned) * sin(self.pi_h_ned) + (y_ned - self.yk_ned) * cos(self.pi_h_ned)

        beta_hat_dot = self.gamma * self.delta * y_e_ned / sqrt(self.delta**2 + y_e_ned**2)
        self.beta_hat += self.h * beta_hat_dot
        psi_d_ned = self.pi_h_ned + atan(self.K_p*y_e_ned) - self.beta_hat

        return x_e_ned, y_e_ned, psi_d_ned
        

