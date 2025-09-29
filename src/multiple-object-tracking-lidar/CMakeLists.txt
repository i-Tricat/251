cmake_minimum_required(VERSION 2.8.3)
project(multi_object_tracking_lidar)  # ← 반드시 package.xml의 <name>과 동일하게!

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs          # ✅ string 포함 메시지 사용 시 필수
  pcl_ros
  laser_geometry
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros laser_geometry
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# ======== 실행 파일 정의 ========
add_executable(kf_tracker src/main.cpp)
add_executable(S_to_P src/S_to_P.cpp)

target_link_libraries(kf_tracker
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(S_to_P
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(kf_tracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(S_to_P
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ======== 설치 경로 지정 ========
install(TARGETS kf_tracker S_to_P
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
