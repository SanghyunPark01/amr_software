#include "amr_ui/handler.h"

AMR_UI_Handler::AMR_UI_Handler(const ros::NodeHandle& nh_):_nh(nh_)
{
    // param
    _nh.param<std::string>("/package_path", THIS_PACKAGE_PATH, "");
        // Map Conversion
    _nh.param<double>("/map_converter/handler/octree_resolution", OCTREE_RESOLUTION, 0.05);
    _nh.param<double>("/map_converter/handler/z_margin_threshold", Z_MARGIN, 0.2);
    _nh.param<bool>("/map_converter/handler/ros_publish", PUB_GRID_MAP, true);

    //
    _mSrvConvertFlag = _nh.advertiseService("/ui/map_converter/bc43cc2993d551056f4ef3801d525ce0", &AMR_UI_Handler::setMapConversion, this);
    _mPub2DGrid = _nh.advertise<nav_msgs::OccupancyGrid>("/girdmap/2d", 1);
    _mPubConversionSatus = _nh.advertise<std_msgs::Float32>("/ui/map_converter_status/5089e6a42f124640607c98bd9cb4c890", 1);
}

bool AMR_UI_Handler::setMapConversion(amr_ui::map_conversion_settingRequest& req, amr_ui::map_conversion_settingResponse& res)
{
    if(req.pcd_path == "stop_bc43cc2993d551056f4ef3801d525ce0" && req.z_value == 2001)
    {
        _mbStartMapConversion = false;
        if(_mptrMapConverter != nullptr)_mptrMapConverter->stop();
        std::cout << "stop!!!\n";
        res.status = true;
        return true;
    }

    _mptrMapConverter = new MapConverter(OCTREE_RESOLUTION, Z_MARGIN, _mPubConversionSatus);
    _mptrMapConverter->setParam(req.pcd_path, req.z_value);

    res.status = true;

    _mbStartMapConversion = true;

    return true;
}

/* @@@@@@@@@@@@@@@@@@@
@@@@@@@ Thread @@@@@@@
@@@@@@@@@@@@@@@@@@@ */

void AMR_UI_Handler::processHandler(void)
{
    while (1)
    {
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);


        // Function: Map Conversion
        if(_mbStartMapConversion)
        {
            // Do it!
            convertMapTo2D();
            // Delete Memory
            _mptrMapConverter->release();
            _mptrMapConverter = nullptr;
            // Reset Flag
            _mbStartMapConversion = false;
        }

        // Other Function...
        if(false)
        {
            //
        }

    }
    
}

/* @@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@ For Thread @@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@ */

// Map Conversion
void AMR_UI_Handler::convertMapTo2D(void)
{
    // Build Octomap
    bool bOctoMapIsReady = _mptrMapConverter->buildOctoMap();

    // Convert map to 2D Occupancy Grid map
    bool bGridMapStatus = false;
    if(bOctoMapIsReady)
    {
        bGridMapStatus = _mptrMapConverter->ConvertOcto2GridMap();
    }

    if(PUB_GRID_MAP && bGridMapStatus)
    {
        auto GridMapTmp = *(_mptrMapConverter->getGridMapPtr());
        _mPub2DGrid.publish(GridMapTmp);
    }
}