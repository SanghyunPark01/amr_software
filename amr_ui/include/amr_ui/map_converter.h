#ifndef MAP_CONVERTER_H
#define MAP_CONVERTER_H

#include "amr_ui/handler_utility.h"

class MapConverter
{
private:
    // param
    double _mdOctreeResolution;
    double _mdMarginZ;
    std::string _msPcdPath;
    double _mdReferZ;
    
    //
    bool _mbStopFlag = false;

    // ros
    ros::Publisher* _mptrPub;
    void publish(double data)
    {
        std_msgs::Float32 tmpMsg;
        tmpMsg.data = data;
        _mptrPub->publish(tmpMsg);
    }

    // Octree
    octomap::OcTree* _mptrOctree = nullptr;

    // Occupancy grid map
    nav_msgs::OccupancyGrid* _mptrGridMap = nullptr;

    // Save Map
    void saveGridMap(std::string SaveDir);

public:
    MapConverter(double Resolution, double Margin, ros::Publisher& pubStatus);
    void setParam(std::string PcdPath, double Z);
    void stop(void){_mbStopFlag = true;}
    void release(void){
        delete _mptrOctree;
        delete _mptrGridMap;
        delete this;
    }

    bool buildOctoMap(void);
    bool ConvertOcto2GridMap(void);
    nav_msgs::OccupancyGrid* getGridMapPtr(void)
    {
        return _mptrGridMap;
    }
};

#endif MAP_CONVERTER_H