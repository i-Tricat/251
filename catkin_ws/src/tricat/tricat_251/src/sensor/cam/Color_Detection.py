import cv2
import numpy as np

# 이미지의 평균 밝기를 조절하는 함수
def mean_brightness(img):
    fixed = 100
    m = cv2.mean(img)
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv2.add(img, scalar)
    return dst

# 조명 보정을 위한 함수
def illumination_correction(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

# 이미지 전처리 함수
def preprocess_image(raw_img, hsv=True, blur=False, brightness=False):
    img = raw_img.copy()
    if brightness:
        img = mean_brightness(img)
    img = illumination_correction(img)
    if blur:
        img = cv2.GaussianBlur(img, (5, 5), 0)
    if hsv:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img

# 히스토그램 평활화를 적용하여 이미지의 밝기를 균등화하는 함수
def equalize_histogram(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    return img_output

# CLAHE를 적용하여 이미지의 대비를 향상시키는 함수
def apply_clahe(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lab_planes = list(cv2.split(lab))
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab_planes[0] = clahe.apply(lab_planes[0])
    lab = cv2.merge(lab_planes)
    img_output = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return img_output

# 주어진 색상 범위에 따라 색상을 선택하는 함수
def select_color(img, color_range, detect_black=False):
    if detect_black:
        selected = cv2.inRange(img, (0, 0, 0), (180, 255, 50))
    else:
        selected = cv2.inRange(img, np.array(color_range[0]), np.array(color_range[1]))
    return selected

# 윤곽선의 주요 점을 계산하는 함수
def contour_points(contour):
    box_left_top = (min(contour[:, 0, 0]), min(contour[:, 0, 1]))
    box_right_bottom = (max(contour[:, 0, 0]), max(contour[:, 0, 1]))
    center_col = int((box_left_top[0] + box_right_bottom[0]) / 2)
    center_row = int((box_left_top[1] + box_right_bottom[1]) / 2)
    return [box_left_top, box_right_bottom], [center_row, center_col]

# 이미지에 윤곽선과 정보를 그리는 함수
def draw_mark(window, contour, vertices, area, box_points, center_point, shape_name, is_target=False):
    color = (0, 255, 0) if is_target else (15, 219, 250)
    caption = "{}".format(shape_name)
    window = cv2.drawContours(window, [contour], -1, color, -1)
    window = cv2.rectangle(window, box_points[0], box_points[1], color, 1)
    window = cv2.circle(window, (center_point[1], center_point[0]), 2, (0, 0, 255), 2)
    window = cv2.putText(window, caption, (box_points[0][0], box_points[0][1] - 10), cv2.FONT_HERSHEY_PLAIN, 1, color, 1, cv2.LINE_AA)
    return window

# 타겟 도형을 감지하는 함수
def detect_target(img, mark_detect_area, target_detect_area):
    detected_shapes = []
    morph_kernel_close = np.ones((9, 9), np.uint8)
    morph_kernel_open = np.ones((5, 5), np.uint8)
    morph_kernel_small = np.ones((3, 3), np.uint8)

    # 닫기 연산 적용
    morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, morph_kernel_close)
    # 열기 연산 적용
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, morph_kernel_open)
    # 작은 커널을 사용한 추가적인 열기 연산 적용
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, morph_kernel_small)

    shape = cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR)

    contours, _ = cv2.findContours(morph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.029 * peri, True)
        area = cv2.contourArea(approx)
        if area < mark_detect_area:
            continue

        vertex_num = len(approx)
        detected = False
        shape_name = "Other"

        # 필터링 조건: 면적과 둘레의 비율
        if peri == 0:  # 둘레가 0인 경우 방지
            continue
        circularity = 4 * np.pi * (area / (peri * peri))
        if circularity < 0.01:  # 원형도가 너무 낮으면 무시, 컨투어가 명확한 도형을 검출(임계값은 조정 가능)
            continue

        if vertex_num == 3 and area >= target_detect_area:
            shape_name = "Triangle"
            detected = True
        elif vertex_num == 4 and area >= target_detect_area:
            shape_name = "Rectangle"
            detected = True
        elif vertex_num == 10 and area >= target_detect_area:
            shape_name = "Star"
            detected = True
        elif vertex_num == 12 and area >= target_detect_area:
            shape_name = "Cross"
            detected = True
        elif vertex_num > 5:
            ellipse = cv2.fitEllipse(contour)
            (center, axes, orientation) = ellipse
            major_axis_length = max(axes)
            minor_axis_length = min(axes)
            if area >= target_detect_area:
                aspect_ratio = major_axis_length / minor_axis_length
                if aspect_ratio >= 0.8 and aspect_ratio <= 1.2:
                    shape_name = "Circle"
                    detected = True
                else:
                    shape_name = "Ellipse"
                    detected = True

        if detected:
            detected_shapes.append((area, contour, shape_name))

    # 상위 2개의 도형 선택
    detected_shapes = sorted(detected_shapes, key=lambda x: x[0], reverse=True)[:2]
    for area, contour, shape_name in detected_shapes:
        box_points, center_point = contour_points(contour)
        shape = draw_mark(shape, contour, len(contour), area, box_points, center_point, shape_name, True)

    return detected_shapes, shape

# 트랙바의 색상 변경 이벤트 핸들러
def on_change_h(value):
    global color_range, detect_black
    if value == 0 and color_range[1][1] == 0 and color_range[1][2] == 0:
        detect_black = True
    else:
        detect_black = False
        color_range[0][0] = value
        color_range[1][0] = value + 10

# 트랙바의 채도 변경 이벤트 핸들러
def on_change_s(value):
    global color_range, detect_black
    if color_range[0][0] == 0 and value == 0 and color_range[1][2] == 0:
        detect_black = True
    else:
        detect_black = False
        color_range[0][1] = value
        color_range[1][1] = 255

# 트랙바의 명도 변경 이벤트 핸들러
def on_change_v(value):
    global color_range, detect_black
    if color_range[0][0] == 0 and color_range[0][1] == 0 and value == 0:
        detect_black = True
    else:
        detect_black = False
        color_range[0][2] = value
        color_range[1][2] = 255

def main():
    global color_range, detect_black
    detect_black = False
    color_range = [[0, 100, 100], [10, 255, 255]]  # 튜플이 아닌 리스트로 변경
    detection_results = []
    detection_threshold = 5
    detection_frame_window = 10
    frame_count = 0
    detection_count = 0
    final_target_detected = False
    detected_shapes_info = []

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open webcam")
        return

    cv2.namedWindow("Input Image")
    cv2.namedWindow("Detected Shapes")
    cv2.namedWindow("Color Adjustment")
    cv2.createTrackbar("Hue", "Color Adjustment", 0, 180, on_change_h)
    cv2.createTrackbar("Saturation", "Color Adjustment", 100, 255, on_change_s)
    cv2.createTrackbar("Value", "Color Adjustment", 100, 255, on_change_v)

    while True:
        ret, raw_img = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Apply histogram equalization, CLAHE and illumination correction to maintain consistent brightness and contrast
        equalized_img = equalize_histogram(raw_img)
        clahe_img = apply_clahe(equalized_img)
        corrected_img = illumination_correction(clahe_img)

        preprocessed = preprocess_image(corrected_img, hsv=True, blur=True, brightness=False)
        hsv_img = select_color(preprocessed, color_range, detect_black)
        
        if detected_shapes_info:
            # Check around previous detection
            prev_shape = detected_shapes_info[0]
            try:
                if isinstance(prev_shape, tuple) and len(prev_shape) >= 2 and isinstance(prev_shape[0], list) and len(prev_shape[0]) >= 2:
                    x, y, w, h = prev_shape[0][0][0], prev_shape[0][0][1], prev_shape[0][1][0] - prev_shape[0][0][0], prev_shape[0][1][1] - prev_shape[0][0][1]
                    roi = hsv_img[y:y+h, x:x+w]
                    targets, shape_img = detect_target(roi, 1000, 2000)
                    if targets:
                        targets[0] = (targets[0][0], targets[0][1] + np.array([[x, y]]), targets[0][2])
                else:
                    targets, shape_img = detect_target(hsv_img, 1000, 2000)
            except (IndexError, TypeError) as e:
                print(f"Error: {e}")
                targets, shape_img = detect_target(hsv_img, 1000, 2000)
        else:
            targets, shape_img = detect_target(hsv_img, 1000, 2000)

        if targets:
            detection_count += 1
            detected_shapes_info = targets
        else:
            detected_shapes_info = []

        detection_results.append(targets)
        if len(detection_results) > detection_frame_window:
            if detection_results.pop(0):
                detection_count -= 1
        frame_count += 1
        if frame_count >= detection_frame_window:
            if detection_count >= detection_threshold:
                final_target_detected = True
            else:
                final_target_detected = False
            frame_count = 0
            detection_count = 0
            detection_results = []

        if 'shape_img' in locals():
            line_position = shape_img.shape[1] // 2
            shape_img = cv2.line(shape_img, (line_position, 0), (line_position, shape_img.shape[0]), (255, 0, 0), 2)

            

            cv2.imshow("Detected Shapes", shape_img)

        cv2.imshow("Input Image", raw_img)

        key = cv2.waitKey(1)
        if key == 27:  # Press 'ESC' to exit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
# hsv 값
#1. red: 170,180,120
#2. black: 0,0,0
#3. blue: 104,103,90
#4. green: 78,100,90 
#5. orange: 2,150,100
#6. yellow: 22,80,100