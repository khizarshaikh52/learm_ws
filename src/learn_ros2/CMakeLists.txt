cmake_minimum_required(VERSION 3.5)
project(learn_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)



ament_package()
