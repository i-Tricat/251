from math import hypot

cur_x = -10.715  # 현재 위치 x
cur_y = -10.988  # 현재 위치 y
wp_x = -10.100   # 웨이포인트 x
wp_y = -9.556    # 웨이포인트 y

distance = hypot(wp_x - cur_x, wp_y - cur_y)
print(f"계산된 거리: {distance:.3f} m")
