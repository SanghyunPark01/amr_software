#ifndef UTILITY_HANDLER_H
#define UTILITY_HANDLER_H

#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>

#include "amr_ui/map_conversion_setting.h"
#include "amr_ui/status.h"

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

std::string THIS_PACKAGE_PATH;

void mySigintHandler(int sig){
    ros::shutdown();
}

#endif UTILITY_HANDLER_H