#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
gate_through_pd_nav.py (충돌 없는 듀얼 모드)
- 기본값: ROS 이미지 토픽 구독(/usb_cam/image_raw) → 장치 충돌 없음
- 옵션: ~use_ros_image:=false 설정 시 OpenCV로 장치 직접 오픈(~camera_device)

출력: /Control (tricat_msgs/Control) thruster_p, thruster_s (UInt16 µs)
디버그: /gate/debug_image, /gate/err_x, /gate/pd_out
"""

import time
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, UInt16
from tricat_msgs.msg import Control

# ===== 전처리/색분할 =====
def mean_brightness(img):
    fixed = 100
    m = cv2.mean(img)
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    return cv2.add(img, scalar)

def motion_blur_correction(img):
    return cv2.bilateralFilter(cv2.GaussianBlur(img, (5, 5), 0), 9, 75, 75)

def illumination_correction(img):
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

def apply_white_balance(img):
    if hasattr(cv2, "xphoto") and hasattr(cv2.xphoto, "createSimpleWB"):
        try:
            return cv2.xphoto.createSimpleWB().balanceWhite(img)
        except Exception:
            pass
    f = img.astype(np.float32)
    b, g, r = f.mean(axis=(0, 1))
    avg = (b + g + r) / 3.0 + 1e-6
    scale = np.array([avg / max(b, 1e-6), avg / max(g, 1e-6), avg / max(r, 1e-6)], np.float32)
    f *= scale
    return np.clip(f, 0, 255).astype(np.uint8)

def adjust_gamma(image, gamma=1.0):
    inv = 1.0 / max(gamma, 1e-6)
    table = (np.arange(256) / 255.0) ** inv * 255.0
    return cv2.LUT(image, table.astype(np.uint8))

def preprocess_image(raw_img, target_color, hsv=True, blur=False, brightness=False):
    img = apply_white_balance(raw_img.copy())
    img = adjust_gamma(img, 1.1)
    if brightness:
        img = mean_brightness(img)
    img = illumination_correction(img)
    img = motion_blur_correction(img)
    if blur:
        img = cv2.GaussianBlur(img, (5, 5), 0)
    if hsv and target_color != 'black':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img

def define_color_range(hmin, hmax, smin=65, smax=255, vmin=45, vmax=255):
    return np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax])

COLOR_RANGES = {
    'red1': define_color_range(0, 10),
    'red2': define_color_range(170, 180),
    'green': define_color_range(25, 100),
}

def select_color(img_hsv_or_bgr, color_range, target_color, thresh=50):
    if target_color == 'black':
        gray = cv2.cvtColor(img_hsv_or_bgr, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
        return mask
    return cv2.inRange(img_hsv_or_bgr, np.array(color_range[0]), np.array(color_range[1]))

def largest_blob(mask, min_area=600):
    if mask is None or mask.size == 0:
        return None
    morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    ret = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = ret[0] if len(ret) == 2 else ret[1]
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    x, y, w, h = cv2.boundingRect(c)
    M = cv2.moments(c)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = x + w // 2, y + h // 2
    return dict(contour=c, area=area, bbox=(x, y, w, h),
                center=(cx, cy), height=h, width=w, morph=morph)

def pixels_to_distance(pix_h, img_h, vfov_deg=50.0, obj_h_m=1.0):
    if pix_h <= 1:
        return None
    vfov = np.deg2rad(vfov_deg)
    f_pix = (img_h / 2.0) / np.tan(vfov / 2.0)
    return float(f_pix * (obj_h_m / pix_h))

def bearing_from_px(cx, img_w, hfov_deg=78.0):
    dx = (cx - img_w / 2.0)
    return float(np.deg2rad(hfov_deg) / img_w * dx)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# ===== 메인 노드 =====
class GateNavigatorPD:
    def __init__(self):
        rospy.init_node("gate_through_pd_nav")
        self.bridge = CvBridge()

        # 퍼블리셔
        self.pub_dbg = rospy.Publisher("/gate/debug_image", Image, queue_size=1)
        self.pub_err = rospy.Publisher("/gate/err_x", Float32, queue_size=10)
        self.pub_u = rospy.Publisher("/gate/pd_out", Float32, queue_size=10)

        topic_left = rospy.get_param("~topic_left", "/Control/thruster_s")
        topic_right = rospy.get_param("~topic_right", "/Control/thruster_p")
        self.pub_left = rospy.Publisher(topic_left, UInt16, queue_size=10)
        self.pub_right = rospy.Publisher(topic_right, UInt16, queue_size=10)

        # 카메라/지오메트리 파라미터
        self.width = int(rospy.get_param("~width", 1280))
        self.height = int(rospy.get_param("~height", 720))
        self.hfov_deg = float(rospy.get_param("~hfov_deg", 78.0))
        self.vfov_deg = float(rospy.get_param("~vfov_deg", 50.0))
        self.obj_h_m = float(rospy.get_param("~buoy_height_m", 1.0))
        self.min_area = int(rospy.get_param("~min_area", 800))

        # 제어 파라미터
        self.kp_px = float(rospy.get_param("~kp_px", 0.0025))
        self.kd_px = float(rospy.get_param("~kd_px", 0.0100))
        self.k_yaw_pwm = float(rospy.get_param("~k_yaw_pwm", 300.0))
        self.fwd_pwm = int(rospy.get_param("~fwd_pwm", 120))
        self.max_delta = int(rospy.get_param("~max_delta_pwm", 400))

        # 상태
        self.prev_err = 0.0
        self.prev_t = None
        self.NEUTRAL = 1500
        self.thruster_p = 0.0
        self.thruster_s = 0.0

        # Control 메시지
        self.control_msg = Control()
        self.control_pub = rospy.Publisher("/Control", Control, queue_size=1)

        # ===== 입력 모드 선택 =====
        self.use_ros_image = bool(rospy.get_param("~use_ros_image", True))
        if self.use_ros_image:
            # ROS 이미지 토픽 구독 (권장)
            self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
            rospy.loginfo(f"[navigator] Subscribing image: {self.image_topic}")
            self.last_frame = None
            self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        else:
            # 장치 직접 오픈 (usb_cam 과 동시 사용 금지)
            self.camera_device = rospy.get_param(
                "~camera_device",
                "/dev/v4l/by-id/usb-SunplusIT_Inc_FHD_Camera_Microphone_01.00.00-video-index0"
            )
            self.use_mjpg = bool(rospy.get_param("~use_mjpg", True))
            self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                rospy.logerr(f"Cannot open camera: {self.camera_device}")
                raise SystemExit(1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            if self.use_mjpg:
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            rospy.loginfo(f"[navigator] Opened device {self.camera_device} {self.width}x{self.height}")

    # ROS 이미지 콜백
    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_frame = frame
        except CvBridgeError as e:
            rospy.logwarn(f"cv_bridge error: {e}")

    def pd_step(self, err_x):
        now = time.time()
        dt = 0.03 if self.prev_t is None else max(1e-3, now - self.prev_t)
        derr = (err_x - self.prev_err) / dt
        u = self.kp_px * err_x + self.kd_px * derr
        self.prev_err, self.prev_t = err_x, now
        self.pub_u.publish(Float32(u))
        return u

    def control_publish(self):
        self.control_msg.thruster_p = UInt16(int(self.thruster_p))
        self.control_msg.thruster_s = UInt16(int(self.thruster_s))
        self.control_pub.publish(self.control_msg)

    def process_frame(self, frame):
        H, W = frame.shape[:2]
        hsv = preprocess_image(frame, target_color='blue', hsv=True, blur=False, brightness=False)

        red = cv2.bitwise_or(
            select_color(hsv, COLOR_RANGES['red1'], 'red'),
            select_color(hsv, COLOR_RANGES['red2'], 'red')
        )
        green = select_color(hsv, COLOR_RANGES['green'], 'green')

        R = largest_blob(red, self.min_area)
        G = largest_blob(green, self.min_area)

        overlay = frame.copy()
        aim_x = None

        if R is not None:
            x, y, w, h = R['bbox']
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(overlay, R['center'], 4, (0, 0, 255), -1)
        if G is not None:
            x, y, w, h = G['bbox']
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(overlay, G['center'], 4, (0, 255, 0), -1)

        if R is not None and G is not None:
            cx = (R['center'][0] + G['center'][0]) // 2
            cy = (R['center'][1] + G['center'][1]) // 2
            aim_x = cx
            cv2.circle(overlay, (cx, cy), 6, (255, 255, 255), -1)
            cv2.line(overlay, (W // 2, 0), (W // 2, H), (255, 0, 0), 1)
            cv2.line(overlay, (cx, 0), (cx, H), (0, 255, 255), 1)
            d_r = pixels_to_distance(R['height'], H, self.vfov_deg, self.obj_h_m) if 'height' in R else None
            d_g = pixels_to_distance(G['height'], H, self.vfov_deg, self.obj_h_m) if 'height' in G else None
            if d_r and d_g:
                br = bearing_from_px(R['center'][0], W, self.hfov_deg)
                bg = bearing_from_px(G['center'][0], W, self.hfov_deg)
                Xr, Zr = np.tan(br) * d_r, d_r
                Xg, Zg = np.tan(bg) * d_g, d_g
                gate_w = np.hypot(Xr - Xg, Zr - Zg)
                cv2.putText(overlay, f"Gate~{gate_w:.1f}m", (10, 24),
                            cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 255, 255), 1, cv2.LINE_AA)

        if aim_x is not None:
            err_x = float(aim_x - W / 2.0)  # +: 목표가 우측
            self.pub_err.publish(Float32(err_x))
            u = self.pd_step(err_x)
            delta = clamp(int(self.k_yaw_pwm * u), -self.max_delta, self.max_delta)
            fwd = self.fwd_pwm
            self.thruster_p = clamp(1500 + fwd - delta, 1100, 1900)
            self.thruster_s = clamp(1500 + fwd + delta, 1100, 1900)
            self.control_publish()
            print(f"{self.thruster_p}, {self.thruster_s}")
            cv2.putText(overlay, f"err:{err_x:.0f}px  u:{u:+.3f}  dPWM:{delta:+d}",
                        (10, H - 12), cv2.FONT_HERSHEY_PLAIN, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
        # else: 필요시 목표 상실 시 정지 로직 추가

        # 디버그 퍼블리시 & 미리보기
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))
        except CvBridgeError:
            pass
        cv2.imshow("Gate PD Navigator", overlay)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.use_ros_image:
                frame = self.last_frame
                if frame is None:
                    rate.sleep()
                    continue
            else:
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    rate.sleep()
                    continue

            self.process_frame(frame)
            rate.sleep()

        if not self.use_ros_image:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    GateNavigatorPD().run()
