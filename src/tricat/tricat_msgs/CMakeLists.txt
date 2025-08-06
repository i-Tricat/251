cmake_minimum_required(VERSION 3.0.2)
project(tricat_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Pose.msg
  Control.msg
  WP.msg
  WPList.msg
  Obstacle.msg
  ObstacleList.msg
  Sensor_total.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  WaypointService.srv
)


generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS 
 std_msgs
 geometry_msgs
 actionlib_msgs
)