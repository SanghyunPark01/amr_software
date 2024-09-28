#ifndef HANDLER_H
#define HANDLER_H

#include "amr_ui/handler_utility.h"
#include "amr_ui/map_converter.h"

class AMR_UI_Handler
{
private:
    // Param
        // Map Converter
    double OCTREE_RESOLUTION;
    double Z_MARGIN;
    bool PUB_GRID_MAP;

    // ROS
    ros::NodeHandle _nh;
        // For Map Converter
    ros::ServiceServer _mSrvConvertFlag;
    ros::Publisher _mPub2DGrid;
    ros::Publisher _mPubConversionSatus;

    // ROS Callback
    bool setMapConversion(amr_ui::map_conversion_settingRequest& req, amr_ui::map_conversion_settingResponse& res);

    // Map Conversion
    MapConverter* _mptrMapConverter = nullptr;
    bool _mbStartMapConversion = false;
    void convertMapTo2D(void);

    // Other Function
    // ...

public:
    AMR_UI_Handler(const ros::NodeHandle& nh_);
    void processHandler(void);
};

#endif HANDLER_H