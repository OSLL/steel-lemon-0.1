cmake_minimum_required(VERSION 2.8)
project(test_robot_detector)

option(WITH_ZBAR "use zbar library as qrcode reader" ON)
option(WITH_NCURSES "use ncurses lib for detector client" ON)

if(WITH_ZBAR)
	find_library(ZBAR_LIB NAMES zbar HINTS ${ARM_LIBS_ROOT})
	find_path(ZBAR_INCLUDE_DIR zbar.h HINTS ${ARM_LIBS_ROOT})
	
	if(NOT ZBAR_LIB OR NOT ZBAR_INCLUDE_DIR)
		message(SEND_ERROR "ZBar library not found")
		return() 
	endif()

	include_directories(${ZBAR_INCLUDE_DIR})	
	add_definitions(-DWITH_ZBAR)
else()
    set(ZBAR_LIB "")
endif()

if(WITH_NCURSES)
    find_path(NCURSES_INCLUDE_DIR ncurses.h HINTS ${ARM_LIBS_ROOT})
  	find_library(NCURSES_LIB NAMES ncurses HINTS ${ARM_LIBS_ROOT})
  	find_library(TINFO_LIB NAME tinfo HINTS ${ARM_LIBS_ROOT})

    if(NOT NCURSES_LIB OR NOT NCURSES_INCLUDE_DIR OR NOT TINFO_LIB)
        message(SEND_ERROR "Ncurses library not found")
        return()
    endif()
    
    include_directories(${NCURSES_INCLUDE_DIR})
    add_definitions(-DWITH_NCURSES)
else()
    set(NCURSES_LIB "")
endif()

include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libqrdetector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_LOGGING")

add_library(brdetector STATIC br_detector.cpp br_detector.h hungarian.c hungarian.h)

#add_executable(detector_node detector_node.cpp)
add_executable(detector_node detector_node_alt.cpp)
add_executable(detector_client detector_client.cpp)

#target_link_libraries(detector_node qrdetector ${Boost_LIBRARIES} ${ROS_LIBS} ${OpenCV_LIBS})
target_link_libraries(detector_node brdetector ${ZBAR_LIB} ${Boost_LIBRARIES} ${ROS_LIBS} ${OpenCV_LIBS})
target_link_libraries(detector_client ${Boost_LIBRARIES} ${ROS_LIBS} ${NCURSES_LIB} ${TINFO_LIB})
