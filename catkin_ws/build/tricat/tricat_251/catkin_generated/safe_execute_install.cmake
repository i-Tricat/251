execute_process(COMMAND "/home/i-tricat241/catkin_ws/build/tricat/tricat_251/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/i-tricat241/catkin_ws/build/tricat/tricat_251/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
