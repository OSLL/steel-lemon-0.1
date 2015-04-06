cmake_minimum_required(VERSION 2.8)

option(CROSS_COMPILE "enable cross-compiling settings" OFF)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" OR CROSS_COMPILE)
	# Build manually on raspberry
	message(STATUS "Using Raspberry Pi build")
	
	execute_process(COMMAND rospack libs-only-L OUTPUT_VARIABLE ROS_LIBS_PATH)
	string(STRIP ${ROS_LIBS_PATH} ROS_LIBS_PATH)
	message(STATUS "ROS libs path: "${ROS_LIBS_PATH})
	
	if(CROSS_COMPILE)
#		set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
#		set(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")
		set(CMAKE_FIND_ROOT_PATH "${ARM_LIBS_ROOT}")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L${ARM_LIBS_ROOT}/usr/lib/")
		set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
		set(CMAKE_INSTALL_RPATH "${ARM_LIBS_ROOT}/usr/lib:${ROS_LIBS_PATH}")
	endif()
	
	find_package(OpenCV REQUIRED)
	find_package(Boost REQUIRED)
	add_subdirectory(src)
	
else()
    # Use rosmake to build this project on x86
    
	find_package(OpenCV REQUIRED)
    add_definitions(-DWITH_NCURSES)
#    rosbuild_add_library(qrdetector STATIC src/libqrdetector/qrdetector.cpp)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
    rosbuild_add_library(jga25_371 STATIC src/drivers/jga_ard.c src/drivers/jga_ard.h)
    rosbuild_add_library(hcsr04 STATIC src/drivers/hcsr04.c src/drivers/hcsr04.h)
    rosbuild_add_library(brdetector STATIC src/detector/br_detector.cpp src/detector/hungarian.c)

    include_directories(./src/libqrdetector ./src/drivers)    
#	rosbuild_add_executable(detector src/detector/detector_node.cpp)
	rosbuild_add_executable(detector_node src/detector/detector_node_alt.cpp)
	rosbuild_add_executable(detector_client src/detector/detector_client.cpp)
	target_link_libraries(detector_node brdetector ${OpenCV_LIBS})
    target_link_libraries(detector_client ncurses)
	
	rosbuild_add_executable(motor_ctrl_node src/motor_ctrl/motor_ctrl_node.cpp)
#	rosbuild_add_executable(motor_client src/motor_ctrl/motor_ctrl_client.cpp)
	target_link_libraries(motor_ctrl_node jga25_371 bcm2835)
	
	rosbuild_add_executable(sonar_node src/sonar/sonar_node.cpp)
	target_link_libraries(sonar_node hcsr04 bcm2835)
	
    rosbuild_add_executable(camera_node src/camera/camera_node.cpp)
	target_link_libraries(camera_node ${OpenCV_LIBS})
	
	rosbuild_add_executable(main_controller src/main_ctrl/main_controller.cpp)
    rosbuild_add_executable(manual_control src/main_ctrl/manual_control.cpp)
endif()

