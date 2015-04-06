cmake_minimum_required(VERSION 2.8)
project(test_robot_main_controller)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(main_controller main_controller.cpp)
add_executable(manual_control manual_control.cpp)
add_executable(cmd_control cmd_control.cpp)

target_link_libraries(main_controller ${Boost_LIBRARIES} ${ROS_LIBS})
target_link_libraries(manual_control ${Boost_LIBRARIES} ${ROS_LIBS})
target_link_libraries(cmd_control ${Boost_LIBRARIES} ${ROS_LIBS})

