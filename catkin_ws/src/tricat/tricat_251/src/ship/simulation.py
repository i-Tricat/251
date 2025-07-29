import matplotlib.pyplot as plt
import yaml

# YAML 데이터를 문자열로 입력
yaml_data = """
header: 
  seq: 740
  stamp: 
    secs: 1740397111
    nsecs:  92166915
  frame_id: "scanner"
segments: 
  - 
    first_point: 
      x: -3.7123793376145704
      y: -1.9217319730171694
      z: 0.0
    last_point: 
      x: -3.5583721625854015
      y: -2.1762128335223148
      z: 0.0
  - 
    first_point: 
      x: -3.581035760546
      y: -2.165961409617859
      z: 0.0
    last_point: 
      x: -3.689149565380899
      y: -2.4819321433740393
      z: 0.0
  - 
    first_point: 
      x: -2.6299508156149174
      y: -2.059586677431948
      z: 0.0
    last_point: 
      x: -2.5106737695130668
      y: -2.094809480724057
      z: 0.0
  - 
    first_point: 
      x: -3.157271791685323
      y: -3.0781783272558068
      z: 0.0
    last_point: 
      x: -2.912707280952285
      y: -3.1672297339476896
      z: 0.0
  - 
    first_point: 
      x: -2.2139137244325258
      y: -2.803194686950207
      z: 0.0
    last_point: 
      x: -1.9546542142289172
      y: -3.122192612612017
      z: 0.0
  - 
    first_point: 
      x: -2.3616917425510975
      y: -3.120855346494405
      z: 0.0
    last_point: 
      x: -2.369196798939493
      y: -3.2738237004467594
      z: 0.0
  - 
    first_point: 
      x: -2.465921507607787
      y: -3.9351173855704253
      z: 0.0
    last_point: 
      x: -2.4154118165275644
      y: -3.9405455510791625
      z: 0.0
  - 
    first_point: 
      x: -1.8535699150058238
      y: -3.133143357251073
      z: 0.0
    last_point: 
      x: -1.816535506605462
      y: -3.5016367838431934
      z: 0.0
  - 
    first_point: 
      x: -1.975524906246853
      y: -3.7697348243762647
      z: 0.0
    last_point: 
      x: -1.951140719577954
      y: -3.7824131442402744
      z: 0.0
  - 
    first_point: 
      x: 0.13204093091048652
      y: -0.5030568145536215
      z: 0.0
    last_point: 
      x: 0.3292186677441306
      y: -0.6020250077315625
      z: 0.0
  - 
    first_point: 
      x: -0.7925148702779913
      y: 1.9553403026387939
      z: 0.0
    last_point: 
      x: -1.0321771441889163
      y: 1.9101374164707754
      z: 0.0
  - 
    first_point: 
      x: -3.150158948381629
      y: 3.875231656348919
      z: 0.0
    last_point: 
      x: -3.1986985273001936
      y: 3.806679871389948
      z: 0.0
  - 
    first_point: 
      x: -3.435992116347584
      y: 3.647414372043542
      z: 0.0
    last_point: 
      x: -3.5784447233588157
      y: 2.829601845917752
      z: 0.0
  - 
    first_point: 
      x: -3.1899220448207544
      y: 2.4691347614285273
      z: 0.0
    last_point: 
      x: -3.573561623689731
      y: 2.4891905897184756
      z: 0.0
  - 
    first_point: 
      x: -3.2904483596094782
      y: 2.247799206022222
      z: 0.0
    last_point: 
      x: -3.7060701327788803
      y: 2.2677074253286165
      z: 0.0
  - 
    first_point: 
      x: -3.9663977216157074
      y: 2.4221181218557453
      z: 0.0
    last_point: 
      x: -4.014431451503778
      y: 2.1119888416119394
      z: 0.0
circles: 
  - 
    center: 
      x: -3.708838037954395
      y: -2.0934304399220047
      z: 0.0
    radius: 0.22173496743270404
  - 
    center: 
      x: -3.7263055461275885
      y: -2.2927370130738542
      z: 0.0
    radius: 0.2428091220661301
  - 
    center: 
      x: -2.580480238826622
      y: -2.111630392289159
      z: 0.0
    radius: 0.12180449357166731
  - 
    center: 
      x: -3.060696460050724
      y: -3.1933037151940593
      z: 0.0
    radius: 0.20026857811655108
  - 
    center: 
      x: -2.176370727420937
      y: -3.03753541481509
      z: 0.0
    radius: 0.2873289768146067
  - 
    center: 
      x: -2.4096024256193265
      y: -3.195173000567156
      z: 0.0
    radius: 0.1384225415461466
  - 
    center: 
      x: -2.4422336382887266
      y: -3.9524123584483464
      z: 0.0
    radius: 0.0793296963006744
  - 
    center: 
      x: -1.941427587576466
      y: -3.3280809820955484
      z: 0.0
    radius: 0.26382151433140505
  - 
    center: 
      x: -1.9669927281669783
      y: -3.7831130918333327
      z: 0.0
    radius: 0.06586745277857417
  - 
    center: 
      x: 0.2020601462648471
      y: -0.609461214040818
      z: 0.0
    radius: 0.1773757584179218
  - 
    center: 
      x: -0.925394954918295
      y: 2.0019233904422484
      z: 0.0
    radius: 0.19080872628924872
  - 
    center: 
      x: -3.1942179312191206
      y: 3.854967931667708
      z: 0.0
    radius: 0.0984954851905688
  - 
    center: 
      x: -3.375952216018059
      y: 2.5899098693674616
      z: 0.0
    radius: 0.2717968495048495
  - 
    center: 
      x: -3.492512238995084
      y: 2.377732972607274
      z: 0.0
    radius: 0.2902344369088824
  - 
    center: 
      x: -4.0799411875470515
      y: 2.280919623512654
      z: 0.0
    radius: 0.23118810305503928
---
목표 위치: x = 2.322, y = 0.133
🚢 현재 위치: x = -0.311, y = 0.345
"""

# YAML 데이터 파싱
parsed_yaml = yaml.safe_load_all(yaml_data)
segments = []
circles = []
current_position = (-6.249, -2.239)
target_position = (2.320, 0.133)

for data in parsed_yaml:
    if data and "segments" in data:
        segments = data["segments"]
    if data and "circles" in data:
        circles = data["circles"]

# 플롯 그리기
fig, ax = plt.subplots(figsize=(10, 8))

# 세그먼트 플롯
for segment in segments:
    x_values = [segment['first_point']['x'], segment['last_point']['x']]
    y_values = [segment['first_point']['y'], segment['last_point']['y']]
    ax.plot(x_values, y_values, 'b-', label='Segments' if segments.index(segment) == 0 else "")

# 원 (Circles) 플롯
for circle in circles:
    center_x, center_y = circle['center']['x'], circle['center']['y']
    radius = circle['radius']
    circle_patch = plt.Circle((center_x, center_y), radius, color='r', fill=False, label='Circles' if circles.index(circle) == 0 else "")
    ax.add_patch(circle_patch)

# 현재 위치 플롯
ax.plot(current_position[0], current_position[1], 'go', markersize=10, label='Current Position')

# 목표 위치 플롯
ax.plot(target_position[0], target_position[1], 'mo', markersize=10, label='Target Position')

# 그래프 설정
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_title('Segments, Circles, Current and Target Positions')
ax.legend()
ax.grid(True)
ax.axis('equal')

plt.show()
