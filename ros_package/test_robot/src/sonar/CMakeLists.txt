cmake_minimum_required(VERSION 2.8)
project(test_robot_sonar)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
set(DRIVER_DIR "${CMAKE_SOURCE_DIR}/src/drivers")

include_directories(${DRIVER_DIR})
add_library(hcsr04 STATIC ${DRIVER_DIR}/hcsr04.c ${DRIVER_DIR}/hcsr04.h)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(sonar_node sonar_node.cpp)
add_executable(sonar_moc sonar_publisher_moc.cpp)
add_dependencies(sonar_node hcsr04)
add_dependencies(sonar_moc hcsr04)

target_link_libraries(sonar_node ${Boost_LIBRARIES} ${ROS_LIBS} hcsr04 bcm2835 rt m)
target_link_libraries(sonar_moc ${Boost_LIBRARIES} ${ROS_LIBS} hcsr04 bcm2835 rt m)

