cmake_minimum_required(VERSION 3.10)
project(mlogging_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required ROS packages
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(udp_server
  src/server/udp_server.cpp
)

# Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(udp_server
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  udp_server
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)