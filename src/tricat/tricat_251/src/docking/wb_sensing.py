#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
gate_through_pd_nav.py
- 빨강/초록 부표를 인식해 두 중심의 정중앙을 목표로 PD 제어
- 출력: 좌/우 쓰러스터 PWM(µs). 정지=1500.
- 디버그: /gate/debug_image, /gate/err_x (px), /gate/pd_out (u)

조정 파라미터(ROS param):
  ~cam_index (int, default 0)
  ~width, ~height (int)
  ~hfov_deg (float, default 78.0)
  ~vfov_deg (float, default 50.0)
  ~buoy_height_m (float, default 1.0)
  ~min_area (int, default 800)
  ~kp_px (float, default 0.0025)       # P 이득 [per pixel]
  ~kd_px (float, default 0.0100)       # D 이득 [per pixel per second]
  ~k_yaw_pwm (float, default 300.0)    # PD 출력(u)을 PWM 차동[µs]로 바꾸는 스케일
  ~fwd_pwm (int, default 120)          # 기본 전진(µs), 1500+fwd_pwm 로 보냄
  ~max_delta_pwm (int, default 400)    # 좌우 차동 한계(±)
  ~topic_left  (str, default "/thruster/left_us")
  ~topic_right (str, default "/thruster/right_us")
"""

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, UInt16
import time

# ===== 전처리/색분할(이전 코드 유지) =====
def mean_brightness(img):
    fixed = 100
    m = cv2.mean(img)
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    return cv2.add(img, scalar)

def motion_blur_correction(img):
    return cv2.bilateralFilter(cv2.GaussianBlur(img, (5,5), 0), 9, 75, 75)

def illumination_correction(img):
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    yuv[:,:,0] = cv2.equalizeHist(yuv[:,:,0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

def apply_white_balance(img):
    if hasattr(cv2, "xphoto") and hasattr(cv2.xphoto, "createSimpleWB"):
        try:
            return cv2.xphoto.createSimpleWB().balanceWhite(img)
        except Exception:
            pass
    f = img.astype(np.float32)
    b,g,r = f.mean(axis=(0,1))
    avg = (b+g+r)/3.0 + 1e-6
    scale = np.array([avg/max(b,1e-6), avg/max(g,1e-6), avg/max(r,1e-6)], np.float32)
    f *= scale
    return np.clip(f,0,255).astype(np.uint8)

def adjust_gamma(image, gamma=1.5):
    inv = 1.0/max(gamma,1e-6)
    table = (np.arange(256)/255.0)**inv * 255.0
    return cv2.LUT(image, table.astype(np.uint8))

def preprocess_image(raw_img, target_color, hsv=True, blur=False, brightness=False):
    img = apply_white_balance(raw_img.copy())
    img = adjust_gamma(img, 1.5)
    if brightness: img = mean_brightness(img)
    img = illumination_correction(img)
    img = motion_blur_correction(img)
    if blur: img = cv2.GaussianBlur(img,(5,5),0)
    if hsv and target_color!='black':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img

def define_color_range(hmin,hmax,smin=50,smax=255,vmin=50,vmax=255):
    return np.array([hmin,smin,vmin]), np.array([hmax,smax,vmax])

COLOR_RANGES = {
    'red1': define_color_range(0,10),
    'red2': define_color_range(170,180),
    'green': define_color_range(115,120),
}

def select_color(img_hsv_or_bgr, color_range, target_color, thresh=50):
    if target_color=='black':
        gray = cv2.cvtColor(img_hsv_or_bgr, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, thresh,255,cv2.THRESH_BINARY_INV)
        return mask
    return cv2.inRange(img_hsv_or_bgr, np.array(color_range[0]), np.array(color_range[1]))
def largest_blob(mask, min_area=600):
    """마스크에서 가장 큰 블롭의 contour/center/bbox/area를 반환. 없으면 None."""
    if mask is None or mask.size == 0:
        return None

    # 노이즈 정리
    morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9,9), np.uint8))
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN,  np.ones((5,5), np.uint8))

    # OpenCV 3/4 호환: findContours 반환 값 처리
    ret = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = ret[0] if len(ret) == 2 else ret[1]

    # 유효 컨투어만 남기기
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    x, y, w, h = cv2.boundingRect(c)

    # 중심: 컨투어 무게중심(안전장치 포함)
    M = cv2.moments(c)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = x + w // 2, y + h // 2  # fallback: 박스 중심

    return dict(contour=c, area=area, bbox=(x, y, w, h),
                center=(cx, cy), height=h, width=w, morph=morph)

def pixels_to_distance(pix_h, img_h, vfov_deg=50.0, obj_h_m=1.0):
    if pix_h<=1: return None
    vfov = np.deg2rad(vfov_deg)
    f_pix = (img_h/2.0)/np.tan(vfov/2.0)
    return float(f_pix * (obj_h_m / pix_h))

def bearing_from_px(cx, img_w, hfov_deg=78.0):
    dx = (cx - img_w/2.0)
    return float(np.deg2rad(hfov_deg) / img_w * dx)

def clamp(v, lo, hi): return lo if v<lo else hi if v>hi else v

# ===== 메인 노드: PD + 쓰러스터 PWM 맵핑 =====
class GateNavigatorPD:
    def __init__(self):
        rospy.init_node("gate_through_pd_nav")
        self.bridge = CvBridge()

        # pubs
        self.pub_dbg  = rospy.Publisher("/gate/debug_image", Image, queue_size=1)
        self.pub_err  = rospy.Publisher("/gate/err_x", Float32, queue_size=10)
        self.pub_u    = rospy.Publisher("/gate/pd_out", Float32, queue_size=10)

        topic_left  = rospy.get_param("~topic_left",  "/thruster/left_us")
        topic_right = rospy.get_param("~topic_right", "/thruster/right_us")
        self.pub_left  = rospy.Publisher(topic_left,  UInt16, queue_size=10)
        self.pub_right = rospy.Publisher(topic_right, UInt16, queue_size=10)

        # cam params
        self.cam_index = rospy.get_param("~cam_index", 0)
        self.width     = rospy.get_param("~width", 1280)
        self.height    = rospy.get_param("~height", 720)
        self.hfov_deg  = rospy.get_param("~hfov_deg", 78.0)
        self.vfov_deg  = rospy.get_param("~vfov_deg", 50.0)
        self.obj_h_m   = rospy.get_param("~buoy_height_m", 1.0)
        self.min_area  = rospy.get_param("~min_area", 800)

        # PD & PWM params
        self.kp_px       = rospy.get_param("~kp_px", 0.0025)
        self.kd_px       = rospy.get_param("~kd_px", 0.0100)
        self.k_yaw_pwm   = rospy.get_param("~k_yaw_pwm", 300.0)  # u -> deltaPWM
        self.fwd_pwm     = int(rospy.get_param("~fwd_pwm", 120)) # 1500+fwd
        self.max_delta   = int(rospy.get_param("~max_delta_pwm", 400))

        # state
        self.prev_err = 0.0
        self.prev_t   = None
        self.NEUTRAL  = 1500

        # cam
        self.cap = cv2.VideoCapture(0)
        if self.width:  self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        if self.height: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open camera index {self.cam_index}")
            raise SystemExit(1)

    def send_pwm(self, left_us, right_us):
        self.pub_left.publish(UInt16(int(left_us)))
        self.pub_right.publish(UInt16(int(right_us)))

    def pd_step(self, err_x):
        now = time.time()
        if self.prev_t is None:
            dt = 0.03
        else:
            dt = max(1e-3, now - self.prev_t)

        derr = (err_x - self.prev_err) / dt
        u = self.kp_px*err_x + self.kd_px*derr  # PD 출력(무차원)
        self.prev_err, self.prev_t = err_x, now
        self.pub_u.publish(Float32(u))
        return u

    def stop_thrusters(self):
        self.send_pwm(self.NEUTRAL, self.NEUTRAL)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ok, frame = self.cap.read()
            if not ok:
                self.stop_thrusters()
                rate.sleep()
                continue

            H,W = frame.shape[:2]
            hsv = preprocess_image(frame, target_color='blue', hsv=True, blur=False, brightness=False)

            # red & green masks
            red = cv2.bitwise_or(
                select_color(hsv, COLOR_RANGES['red1'], 'red'),
                select_color(hsv, COLOR_RANGES['red2'], 'red')
            )
            green = select_color(hsv, COLOR_RANGES['green'], 'green')

            R = largest_blob(red,   self.min_area)
            G = largest_blob(green, self.min_area)

            overlay = frame.copy()
            aim_x = None

            if R is not None:
                x,y,w,h = R['bbox']
                cv2.rectangle(overlay,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.circle(overlay, R['center'], 4, (0,0,255), -1)
            if G is not None:
                x,y,w,h = G['bbox']
                cv2.rectangle(overlay,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.circle(overlay, G['center'], 4, (0,255,0), -1)

            if R is not None and G is not None:
                cx = (R['center'][0] + G['center'][0])//2
                cy = (R['center'][1] + G['center'][1])//2
                aim_x = cx
                cv2.circle(overlay,(cx,cy),6,(255,255,255),-1)
                cv2.line(overlay,(W//2,0),(W//2,H),(255,0,0),1)
                cv2.line(overlay,(cx,0),(cx,H),(0,255,255),1)
                # 참고 정보(거리/간격)
                d_r = pixels_to_distance(R['height'], H, self.vfov_deg, self.obj_h_m) if 'height' in R else None
                d_g = pixels_to_distance(G['height'], H, self.vfov_deg, self.obj_h_m) if 'height' in G else None
                if d_r and d_g:
                    br = bearing_from_px(R['center'][0], W, self.hfov_deg)
                    bg = bearing_from_px(G['center'][0], W, self.hfov_deg)
                    Xr, Zr = np.tan(br)*d_r, d_r
                    Xg, Zg = np.tan(bg)*d_g, d_g
                    gate_w = np.hypot(Xr-Xg, Zr-Zg)
                    cv2.putText(overlay, f"Gate~{gate_w:.1f}m", (10,24),
                                cv2.FONT_HERSHEY_PLAIN, 1.2, (0,255,255), 1, cv2.LINE_AA)

            if aim_x is not None:
                err_x = float(aim_x - W/2.0)  # +면 목표가 우측
                self.pub_err.publish(Float32(err_x))

                # === PD ===
                u = self.pd_step(err_x)       # 무차원 조타요구
                delta = clamp(int(self.k_yaw_pwm * u), -self.max_delta, self.max_delta)

                fwd = self.fwd_pwm
                left_us  = clamp(1500 + fwd - delta, 1100, 1900)
                right_us = clamp(1500 + fwd + delta, 1100, 1900)
                self.send_pwm(left_us, right_us)

                cv2.putText(overlay, f"err:{err_x:.0f}px  u:{u:+.3f}  dPWM:{delta:+d}",
                            (10,H-12), cv2.FONT_HERSHEY_PLAIN, 1.3, (0,255,0), 1, cv2.LINE_AA)
            else:
                # 목표 상실: 안전 정지
                self.stop_thrusters()

            # debug publish & preview
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))
            cv2.imshow("Gate PD Navigator", overlay)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    GateNavigatorPD().run()
