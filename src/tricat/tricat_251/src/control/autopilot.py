import numpy as np
from math import atan2, degrees, pi, sqrt


###################################################################################################################################
#
# ROS 사용 부분
# 
###################################################################################################################################


# Point tracking Method
def heading_cal(x_target_ned, y_target_ned, x_ned, y_ned):
    '''
    Input:
        x_target_ned ():
        y_target_ned ():
        x_ned ():
        y_ned ():
    
    Output:
        psi_d_ned (): [rad]
    '''
    psi_d_ned = atan2(float(y_target_ned) - y_ned, float(x_target_ned) - x_ned)
    return psi_d_ned

#TODO: PID 제어 기법을 이렇게 사용해도 되는지 확인 & setpoint 확인 방법 생각
class HeadingAutoPilot:
    '''
        psi_d를 받아서 delta_c를 내보냄
        delta_c = -K_p*(psi-psi_d)-K_d*r [rad]
    '''
    def __init__(self, kp_servo, kd_servo, psi_d):
        self.psi_d = psi_d
        self.kp = kp_servo
        self.kd = kd_servo
        self.delta_c = 0 

    def update(self, psi, r_ned):
        self.delta_c = -self.kp*(psi-self.psi_d)-self.kd*r_ned
        return self.delta_c

    def set_psi_d(self, psi_d):
        self.psi_d = psi_d

###################################################################################################################################
#
# Otter 운동 모델을 위한 부분
#
###################################################################################################################################

# [x_d,v_d,a_d] = refModel3(x_d,v_d,a_d,r,wn_d,zeta_d,v_max,sampleTime) is a 3-order 
# reference  model for generation of a smooth desired position x_d, velocity |v_d| < v_max, 
# and acceleration a_d. Inputs are natural frequency wn_d and relative damping zeta_d.
def refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime):
    
    # desired "jerk"
    j_d = wn_d**3 * (r -x_d) - (2*zeta_d+1) * wn_d**2 * v_d - (2*zeta_d+1) * wn_d * a_d

   # Forward Euler integration
    x_d += sampleTime * v_d             # desired position
    v_d += sampleTime * a_d             # desired velocity
    a_d += sampleTime * j_d             # desired acceleration 
    
    # Velocity saturation
    if (v_d > v_max):
        v_d = v_max
    elif (v_d < -v_max): 
        v_d = -v_max    
    
    return x_d, v_d, a_d


# SISO PID pole placement
def PIDpolePlacement(
    e_int,
    e_x,
    e_v,
    x_d,
    v_d,
    a_d,
    m,
    d,
    k,
    wn_d,
    zeta_d,
    wn,
    zeta,
    r,
    v_max,
    sampleTime,
):

    # PID gains based on pole placement
    Kp = m * wn ** 2.0 - k
    Kd = m * 2.0 * zeta * wn - d
    Ki = (wn / 10.0) * Kp

    # PID control law
    u = -Kp * e_x - Kd * e_v - Ki * e_int

    # Integral error, Euler's method
    e_int += sampleTime * e_x

    # 3rd-order reference model for smooth position, velocity and acceleration
    [x_d, v_d, a_d] = refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime)

    return u, e_int, x_d, v_d, a_d


def controlAllocation(Binv, tau_X, tau_N):
        """
        [n1, n2] = controlAllocation(tau_X, tau_N)
        """
        tau = np.array([tau_X, tau_N])  # tau = B * u_alloc
        u_alloc = np.matmul(Binv, tau)  # u_alloc = inv(B) * tau

        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * sqrt(abs(u_alloc[0]))
        n2 = np.sign(u_alloc[1]) * sqrt(abs(u_alloc[1]))

        return n1, n2
