cmake_minimum_required(VERSION 3.0.2)
project(amr_ui)

set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


add_service_files(
  DIRECTORY srv
  FILES
  map_conversion_setting.srv
  status.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES amr_ui
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
)

catkin_install_python(PROGRAMS
  scripts/main_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(${PROJECT_NAME}_ui_handler src/handler.cpp)
target_link_libraries(${PROJECT_NAME}_ui_handler
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_map_converter src/map_converter.cpp)
target_link_libraries(${PROJECT_NAME}_map_converter
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(amr_ui_handler_node
  src/main.cpp
)
target_link_libraries(amr_ui_handler_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_ui_handler
  ${PROJECT_NAME}_map_converter
)