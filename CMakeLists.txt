project(cudaPcl)
cmake_minimum_required(VERSION 2.4.6)

set(POD_NAME cudaPcl)
include(cmake/pods.cmake)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_COLOR_MAKEFILE ON)
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug) # Release; debug; ...
  set(CMAKE_BUILD_TYPE Release) # Release; debug; ...
ENDIF()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )
endif()

# add flags depending on build type
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -Wno-deprecated -O2 -finline-functions -DNDEBUG -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wno-deprecated -std=c++11")

find_package (Eigen3 REQUIRED)
#find_package (Boost 1.54 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (PCL REQUIRED)
#find_package (VTK REQUIRED)
#message(${EIGEN3_INCLUDE_DIR})
#message(${Boost_LIBRARY_DIRS})
#message("VTK ")
#message(${VTK_LIBRARY_DIRS})
#message(${VTK_INCLUDE_DIRS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  ${VTK_INCLUDE_DIRS}
#  /usr/include/
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  /usr/lib/
)

add_executable(realSenseGrabber
  ./src/realSenseGrabber.cpp
  )
target_link_libraries (realSenseGrabber
  opencv_core opencv_highgui opencv_contrib opencv_imgproc
  realsense
  )

pods_install_headers(
  ./include/rgbdGrabber/realSenseGrabber.hpp
  DESTINATION rgbdGrabber)

