cmake_minimum_required(VERSION 3.4.1)
project(mlogging)

###################################
### CMake flags
###################################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set ( PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
configure_file(
	"${PROJECT_SOURCE_DIR}/MloggingConfig.h.in"
	"${PROJECT_BINARY_DIR}/MloggingConfig.h"
)
###################################
### Packages
###################################

find_package(PCL REQUIRED)

if(PCL_FOUND)
	message("Found PCL")
endif()


include_directories(include build )
add_executable(mlogging src/mlogging.cpp)

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)


