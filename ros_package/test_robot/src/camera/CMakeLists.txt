cmake_minimum_required(VERSION 2.8)
project(test_robot_camera)

include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(camera_node camera_node.cpp)

target_link_libraries(camera_node ${Boost_LIBRARIES} ${ROS_LIBS} ${OpenCV_LIBS})

