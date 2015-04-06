cmake_minimum_required(VERSION 2.8)

project(qrdetector)

option(WITH_ZBAR "use zbar library as qrcode reader" OFF)
option(WITH_QR_RECOGNITION "enable qr recognition and rebuilding" OFF)
option(WITH_DEBUG_OUTPUT "enable debug output" OFF)

if(NOT DEFINED OpenCV_LIBS)
	find_package(OpenCV REQUIRED)
endif()

if(WITH_ZBAR)
	find_library(ZBAR_LIB NAMES zbar libzbar)
	find_path(ZBAR_INCLUDE_DIR zbar.h)
	
	if(NOT ZBAR_LIB OR NOT ZBAR_INCLUDE_DIR)
		message(SEND_ERROR "ZBar library not found")
		return() 
	endif()

	include_directories(${ZBAR_INCLUDE_DIR})	
	add_definitions(-DWITH_ZBAR)
endif()

if(WITH_QR_RECOGNITION)
	add_definitions(-DWITH_QR_RECOGNITION)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(qrdetector STATIC qrdetector.cpp qrdetector.h)

if(WITH_ZBAR)
	target_link_libraries(qrdetector ${ZBAR_LIB})
	if(WIN32)
		target_link_libraries(qrdetector iconv)
	endif()
endif()

