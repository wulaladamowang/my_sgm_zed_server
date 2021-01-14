SET(execName my_sgm_zed)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

IF(NOT MSVC)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)



include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(../../include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*)
FILE(GLOB_RECURSE HDR_FILES include/*)

cuda_add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
add_definitions(-std=c++11 -g -O3 -pthread)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${execName} sgm ${ZED_LIBS} ${SPECIAL_OS_LIBS} ${OpenCV_LIBRARIES})
