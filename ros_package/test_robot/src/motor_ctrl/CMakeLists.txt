cmake_minimum_required(VERSION 2.8)
project(test_robot_motor_controller)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRH_COORDINATES")
set(MOTOR_DRIVER_DIR "${CMAKE_SOURCE_DIR}/src/drivers")

include_directories(${MOTOR_DRIVER_DIR})
add_library(jga25_371 STATIC ${MOTOR_DRIVER_DIR}/jga_ard.c ${MOTOR_DRIVER_DIR}/jga_ard.h)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(motor_ctrl_node motor_ctrl_node.cpp)
add_executable(motor_ctrl_sub motor_ctrl_node_sub.cpp)
add_dependencies(motor_ctrl_node jga25_371)
add_dependencies(motor_ctrl_sub jga25_371)

target_link_libraries(motor_ctrl_node ${Boost_LIBRARIES} ${ROS_LIBS} jga25_371 bcm2835 m)
target_link_libraries(motor_ctrl_sub ${Boost_LIBRARIES} ${ROS_LIBS} jga25_371 bcm2835 m)
