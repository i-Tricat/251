#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

class SteeringController:
    '''
    A class to control the steering mechanism with rudder limits and rate limits.
    
    Attributes:
        delta_max (float): maximum rudder angle in degrees
        delta_dot_max (float): maximum rudder rate in degrees per second
        delta_c (int):
        dt (float): sampling time in seconds
    
    Methods:
        rudder_limiter(delta_c):
            Limits the commanded rudder angle to within the maximum rudder angle.
        
        rudder_rate_limiter(delta_c):
            Limits the rate of change of the rudder angle to within the maximum rudder rate.
        
        compute(delta_c):
            Computes the new rudder angle based on the commanded delta, considering the limits.
    '''
    
    def __init__(self, delta_max, delta_dot_max, delta_c, sample_time = 0.1):
        self.delta_max = delta_max
        self.delta_dot_max = delta_dot_max
        self.delta_c = delta_c
        self.sample_time = sample_time
        self.delta_prev = delta_c
        
    def rudder_limiter(self, delta_c):
        return np.clip(delta_c, self.delta_max[0], self.delta_max[1])

    def rudder_rate_limiter(self, delta_c):
        delta_dot = (delta_c - self.delta_prev) / self.sample_time
        if delta_dot > self.delta_dot_max:
            delta_c = self.delta_prev + self.delta_dot_max * self.sample_time
        elif delta_dot < -self.delta_dot_max:
            delta_c = self.delta_prev - self.delta_dot_max * self.sample_time
        return delta_c
    
    def compute(self, delta_c):
        '''
        Computes the limited rudder angle based on the commanded delta.

        Input:
            delta_c (float): commanded delta
        
        Output:
            float: the new rudder angle after applying limits
        '''
        limited_delta_c = self.rudder_limiter(delta_c)
        self.delta_c = self.rudder_rate_limiter(limited_delta_c)
        self.delta_prev = self.delta_c
        return self.delta_c

class SpeedController:
    def __init__(self, kp_thruster, ki_thruster, setpoint, kd_thruster = 0, sample_time = 0.1):
        self.pid = Pid(kp_thruster, ki_thruster, kd_thruster, setpoint, sample_time)
        self.setpoint = setpoint
        self.feedback = 0
    
    def update(self, feedback):
        self.feedback = self.pid.update(feedback)
        return self.feedback

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.pid.setSetpoint(setpoint)    

class Pid:
    """
    PID controller.
    
    Input:
        proportional gain, Kp
        integral gain, Ki
        Kd
        setpoint
        sample_time: 0.1 (10Hz)
    """
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self.Cp = 0.0
        self.Ci = 0.0 
        self.Cd = 0.0
        self.previous_error = 0.0

    def update(self, feedback):
        error = self.setpoint - feedback

        self.Cp = error
        self.Ci += error * self.sample_time
        self.Cd = (error - self.previous_error) / self.sample_time

        output = (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )

        self.previous_error = error

        return output

    def setKp(self, Kp):
        self.Kp = Kp

    def setKi(self, Ki):
        self.Ki = Ki 

    def setKd(self, Kd):
        self.Kd = Kd 

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


PWM_RPS = {
    1500: 0,
    1600: 19,
    1700: 33,
    1800: 46
}