cmake_minimum_required(VERSION 3.0.2)
project(livox_laser_simulation)

## Set C++ standard to C++17 to resolve `std::optional` and `std::any` issues
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  livox_ros_driver
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Ensure you are including the correct gazebo version
include_directories(/usr/include/gazebo-7/gazebo) # Update this line if you're using a different version

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES livox_laser_simulation
  CATKIN_DEPENDS tf
)

## Add sources and library
add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)

## Link against gazebo and other necessary libraries
target_link_libraries(livox_laser_simulation
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
  libprotobuf.so.9
)

