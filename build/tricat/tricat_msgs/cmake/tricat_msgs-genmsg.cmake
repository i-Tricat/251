# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tricat_msgs: 7 messages, 1 services")

set(MSG_I_FLAGS "-Itricat_msgs:/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tricat_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" "std_msgs/Header:std_msgs/Float64"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" "std_msgs/Header:std_msgs/UInt16"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" "std_msgs/String:std_msgs/UInt16:std_msgs/Float64:std_msgs/Bool"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" "tricat_msgs/WP:std_msgs/Float64:std_msgs/Header:std_msgs/String:std_msgs/Bool:std_msgs/UInt16"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" "std_msgs/Header:geometry_msgs/Point:tricat_msgs/Obstacle"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" "std_msgs/Header:geometry_msgs/Point:std_msgs/Float64"
)

get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_custom_target(_tricat_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tricat_msgs" "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" "tricat_msgs/WP:std_msgs/Float64:std_msgs/Header:std_msgs/String:std_msgs/Bool:tricat_msgs/WPList:std_msgs/UInt16"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)

### Generating Services
_generate_srv_cpp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
)

### Generating Module File
_generate_module_cpp(tricat_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tricat_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tricat_msgs_generate_messages tricat_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_cpp _tricat_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tricat_msgs_gencpp)
add_dependencies(tricat_msgs_gencpp tricat_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tricat_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)
_generate_msg_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)

### Generating Services
_generate_srv_eus(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
)

### Generating Module File
_generate_module_eus(tricat_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tricat_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tricat_msgs_generate_messages tricat_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_eus _tricat_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tricat_msgs_geneus)
add_dependencies(tricat_msgs_geneus tricat_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tricat_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)
_generate_msg_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)

### Generating Services
_generate_srv_lisp(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
)

### Generating Module File
_generate_module_lisp(tricat_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tricat_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tricat_msgs_generate_messages tricat_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_lisp _tricat_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tricat_msgs_genlisp)
add_dependencies(tricat_msgs_genlisp tricat_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tricat_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)
_generate_msg_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)

### Generating Services
_generate_srv_nodejs(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
)

### Generating Module File
_generate_module_nodejs(tricat_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tricat_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tricat_msgs_generate_messages tricat_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_nodejs _tricat_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tricat_msgs_gennodejs)
add_dependencies(tricat_msgs_gennodejs tricat_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tricat_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)
_generate_msg_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)

### Generating Services
_generate_srv_py(tricat_msgs
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
)

### Generating Module File
_generate_module_py(tricat_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tricat_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tricat_msgs_generate_messages tricat_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Control.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WP.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/WPList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/ObstacleList.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/msg/Sensor_total.msg" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/i-tricat241/catkin_ws/src/tricat/tricat_msgs/srv/WaypointService.srv" NAME_WE)
add_dependencies(tricat_msgs_generate_messages_py _tricat_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tricat_msgs_genpy)
add_dependencies(tricat_msgs_genpy tricat_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tricat_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tricat_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tricat_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tricat_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(tricat_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tricat_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tricat_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tricat_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(tricat_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tricat_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tricat_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tricat_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(tricat_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tricat_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tricat_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tricat_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(tricat_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tricat_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tricat_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tricat_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(tricat_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
