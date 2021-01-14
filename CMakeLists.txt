cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CUDA_ARCH "-arch=sm_75" CACHE STRING "Value of the NVCC -arch option.")
if(${CMAKE_VERSION} VERSION_EQUAL 3.7 OR ${CMAKE_VERSION} VERSION_GREATER 3.7)
  option(AUTO_DETECT_ARCH "Detect local GPU compute arch automatically" ON)
endif()



set(ZED_SDK_LIB "/usr/local/zed/lib/libsl_zed.so" CACHE STRING "ZED SDK library(sl_zed**.llb) path.")
set(ZED_SDK_INCLUDE_DIR "/usr/local/zed/include" CACHE STRING "ZED SDK include path.")


project(libSGM VERSION 2.7.0)

if(BUILD_OPENCV_WRAPPER)
	find_package(OpenCV REQUIRED core)
	include_directories(${OpenCV_INCLUDE_DIRS})
endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

configure_file(${PROJECT_SOURCE_DIR}/include/libsgm_config.h.in
               ${PROJECT_SOURCE_DIR}/include/libsgm_config.h
)

add_subdirectory(src)

add_subdirectory(my_sgm_zed)

