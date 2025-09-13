# Tricat Autonomous Navigation Project (251)

## 1. 프로젝트 개요

본 프로젝트는 ROS(Robot Operating System)를 기반으로 하는 자율 운항 선박(Tricat) 제어 시스템입니다. LiDAR, IMU 등 다양한 센서 데이터를 융합하여 주변 환경을 인지하고, 장애물을 회피하며 목표 지점까지 안전하게 항해하는 것을 목표로 합니다.

## 2. 주요 기술 스택

- **Framework**: ROS(Robot Operating System)
- **Languages**: C++, Python
- **Libraries**:
    - PCL (Point Cloud Library): 3D 포인트 클라우드 처리
    - Eigen3: 행렬 및 벡터 연산
- **주요 ROS 패키지**:
    - **`tricat_251`**: 본 프로젝트의 메인 패키지로, 자율 운항을 위한 제어, 항법, 인지, 도킹 등 핵심 로직을 포함합니다.
    - **`myahrs_driver`**: WithRobot 사의 myAHRS+ IMU 센서 드라이버입니다. 선박의 자세(Roll, Pitch, Yaw) 및 가속도, 각속도 데이터를 제공합니다.
    - **`rplidar_ros`**: Slamtec 사의 RPLiDAR 센서 드라이버입니다. 주변 환경을 2D/3D로 스캔하여 포인트 클라우드 데이터를 생성합니다.
    - **`obstacle_detector`**: LiDAR의 `/scan` 토픽 데이터를 입력받아 원, 선분 등의 형태로 장애물을 감지하고 `/Obstacles` 토픽으로 발행합니다.
    - **`rospy`, `roscpp`**: ROS의 Python 및 C++ 클라이언트 라이브러리입니다.
    - **`sensor_msgs`, `geometry_msgs`, `std_msgs`**: 센서 데이터, 기하학적 정보 등 표준 ROS 메시지 타입을 제공합니다.
    - **`tf`**: 여러 좌표계 간의 관계를 관리하고 변환하는 데 사용됩니다.

## 3. 설치 방법

**전제 조건:**
- Ubuntu 18.04 Bionic Beaver
- ROS Melodic Morenia

1.  **Catkin 작업 공간 생성 및 소스 코드 클론:**
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    # git clone <repository_url> .
    ```

2.  **ROS 의존성 패키지 설치:**
    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **프로젝트 빌드:**
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

4.  **환경 설정:**
    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## 4. 실행 방법

프로젝트의 핵심 기능을 실행하기 위한 기본 launch 파일은 다음과 같습니다.

```bash
roslaunch tricat_251 total.launch
```

`total.launch` 파일은 센서 드라이버, 장애물 감지, 제어 노드 등 시스템 전체를 실행하는 역할을 합니다. (상세 내용은 추후 보강 예정)

## 5. 디렉토리 구조

`src/tricat/tricat_251` 디렉토리의 주요 모듈과 역할은 다음과 같습니다.

```
tricat_251/
└── src/
    ├── actuator/      # 서보, 스러스터 등 액추에이터 제어
    ├── control/       # 경로 추종을 위한 Autopilot 및 PID 제어기
    ├── docking/       # 카메라 비전을 이용한 자동 도킹
    ├── guidance/      # Waypoint 서비스 제공 및 경로 관리
    ├── navigation/    # 센서 데이터 기반 위치/속도 추정
    ├── sensor/        # GPS, IMU, LiDAR, 카메라 등 센서 데이터 처리
    ├── ship/          # 선박 모델, 시뮬레이션 및 Waypoint 관리
    └── visual/        # 데이터 시각화
```

## 6. 핵심 기능

### - 경로 추종 (Waypoint Following)

- **`guidance/guidance.py`**: YAML 파일에 정의된 GPS 좌표(경유지)를 읽어와 ROS 서비스 서버(`get_waypoints`)를 실행합니다.
- **`ship/wp_manager.py`**: 서비스 클라이언트로서 전체 경유지 목록을 받아오고, 선박의 현재 위치를 기준으로 따라가야 할 경유지(`WP_k`)를 동적으로 관리합니다.
- **`control/autopilot.py`**: 목표 경유지와 현재 선박의 상태(위치, 자세)를 비교하여 목표 선수각(heading)을 계산하고, PD 제어기를 통해 서보 모터를 제어하여 경로를 이탈하지 않도록 합니다.

### - 상태 추정 (State Estimation)

- **`navigation/navigation.py`**: `/sensor_total` 토픽으로 들어오는 융합된 센서 데이터를 기반으로 선박의 선형 속도(`u`, `v`) 및 각속도(`r`)를 계산합니다. 최종적으로 추정된 선박의 모든 상태 정보(x, y, psi, u, v, r)를 `/Pose` 토픽으로 발행합니다.

### - 자동 도킹 (Automatic Docking)

- **`docking/Docking.py`**: 카메라 이미지를 받아 특정 색상과 모양의 마커를 탐지합니다. 탐지된 마커의 이미지상 위치를 기반으로 서보와 스러스터를 정밀 제어하여 지정된 위치에 자동으로 도킹을 수행합니다.

---

*이 문서는 프로젝트 분석을 통해 자동으로 생성된 초안이며, 지속적으로 업데이트될 예정입니다.*
