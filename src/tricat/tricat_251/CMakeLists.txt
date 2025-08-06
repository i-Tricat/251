cmake_minimum_required(VERSION 3.0.2)
project(tricat_251)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
  tricat_msgs
  dynamic_reconfigure
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

## ROS dynamic reconfigure parameters ##
generate_dynamic_reconfigure_options(
  cfg/LidarController.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare C++ executables ##
add_executable(3d_lidar_node src/sensor/lidar/3d_lidar.cpp)
add_executable(costmap_node src/navigation/costmap.cpp)

## Link libraries ##
target_link_libraries(3d_lidar_node ${catkin_LIBRARIES})
target_link_libraries(costmap_node ${catkin_LIBRARIES})

## Add dependencies ##
add_dependencies(3d_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Install targets (if needed) can be added here later if necessary
