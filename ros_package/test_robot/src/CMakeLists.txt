cmake_minimum_required(VERSION 2.8)
project(test_robot)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../srv_gen/cpp/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../msg_gen/cpp/include)

set(ROS_LIBS ${ROS_LIBS_PATH}/libroscpp.so ${ROS_LIBS_PATH}/libroscpp_serialization.so ${ROS_LIBS_PATH}/librosconsole.so ${ROS_LIBS_PATH}/librostime.so ${ROS_LIBS_PATH}/libcv_bridge.so ${ROS_LIBS_PATH}/libtf.so)

add_subdirectory(camera)
#add_subdirectory(libqrdetector)
add_subdirectory(detector)
add_subdirectory(motor_ctrl)
add_subdirectory(main_ctrl)
add_subdirectory(sonar)

add_dependencies(detector_node qrdetector)
add_dependencies(main_controller detector_node motor_ctrl_node)
