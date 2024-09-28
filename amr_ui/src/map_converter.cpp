#include "amr_ui/map_converter.h"

MapConverter::MapConverter(double resolution, double margin, ros::Publisher& pubStatus)
{
    _mdOctreeResolution = resolution;
    _mdMarginZ = margin;

    _mptrPub = &pubStatus;
}

void MapConverter::setParam(std::string PcdPath, double Z)
{
    _msPcdPath = PcdPath;
    _mdReferZ = Z;
}

bool MapConverter::buildOctoMap(void)
{
    if(_mbStopFlag)return false;

    // Read .pcd
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::io::loadPCDFile<pcl::PointXYZ> (_msPcdPath, pclCloud);
    double dTotalPointNum = pclCloud.points.size();
    this->publish(10);

    // .pcd to OctoMap
    _mptrOctree = new octomap::OcTree(_mdOctreeResolution);

    // Load point cloud to Octree
    double dCurrNum = 0;
    for(auto point: pclCloud.points)
    {
        _mptrOctree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
        dCurrNum++;
        if((int)dCurrNum % (int)(dTotalPointNum/100) == 0)
        {
            this->publish(10 + 70*(dCurrNum/dTotalPointNum));
            if(_mbStopFlag)return false;
        }
    }
    if(_mbStopFlag)return false;

    // Update Ocotmap
    _mptrOctree->updateInnerOccupancy();
    
    if(_mbStopFlag)return false;

    this->publish(80);

    return true;
}

bool MapConverter::ConvertOcto2GridMap(void)
{
    if(_mptrOctree == nullptr)return false;
    if(_mbStopFlag)return false;

    _mptrGridMap = new nav_msgs::OccupancyGrid();
    _mptrGridMap->info.resolution = _mdOctreeResolution;

    // Check Boundary
    double dMinX, dMinY, dMinZ;
    double dMaxX, dMaxY, dMaxZ;
    double dTotalSize = _mptrOctree->getNumLeafNodes();
    _mptrOctree->getMetricMin(dMinX, dMinY, dMinZ);
    _mptrOctree->getMetricMax(dMaxX, dMaxY, dMaxZ);
    octomap::point3d otMinPt(dMinX, dMinY, dMinZ);
    octomap::point3d otMaxPt(dMaxX, dMaxY, dMaxZ);

    // Set 2D Grid Map Size
    _mptrGridMap->info.width = static_cast<unsigned int>((otMaxPt.x() - otMinPt.x()) / _mdOctreeResolution);
    _mptrGridMap->info.height = static_cast<unsigned int>((otMaxPt.y() - otMinPt.y()) / _mdOctreeResolution);
    
    // Set Origin
    _mptrGridMap->info.origin.position.x = otMinPt.x();
    _mptrGridMap->info.origin.position.y = otMinPt.y();
    _mptrGridMap->info.origin.position.z = 0.0;

    // Initialize -1(unknown)
    _mptrGridMap->data.resize(_mptrGridMap->info.width * _mptrGridMap->info.height, -1);

    // Set Z range
    double zMin = _mdReferZ - _mdMarginZ;
    double zMax = _mdReferZ + _mdMarginZ;

    // Octomap to 2D Occupancy GridMap
    double dCurrNum = 0;

    if(_mbStopFlag)return false;
    for (octomap::OcTree::leaf_bbx_iterator it = _mptrOctree->begin_leafs_bbx(otMinPt, otMaxPt), end = _mptrOctree->end_leafs_bbx(); it != end; ++it) {
        // Get Coordinate
        octomap::point3d point = it.getCoordinate();
        // Check Z value
        if (point.z() >= zMin && point.z() <= zMax) {
            // If Occupied
            if (_mptrOctree->isNodeOccupied(*it)) {
                // Get X, Y
                int nXindex = static_cast<int>((point.x() - otMinPt.x()) / _mdOctreeResolution);
                int nYindex = static_cast<int>((point.y() - otMinPt.y()) / _mdOctreeResolution);
                if (nXindex >= 0 && nXindex < _mptrGridMap->info.width && nYindex >= 0 && nYindex < _mptrGridMap->info.height) {
                    int index = nYindex * _mptrGridMap->info.width + nXindex;
                    _mptrGridMap->data[index] = 100;
                }
            }
            // Not Occupied (but, there is no Not Occupied point because map data is pre-mapping data)
            else
            {
                // Get X, Y
                int nXindex = static_cast<int>((point.x() - otMinPt.x()) / _mdOctreeResolution);
                int nYindex = static_cast<int>((point.y() - otMinPt.y()) / _mdOctreeResolution);
                if (nXindex >= 0 && nXindex < _mptrGridMap->info.width && nYindex >= 0 && nYindex < _mptrGridMap->info.height) {
                    int index = nYindex * _mptrGridMap->info.width + nXindex;
                    _mptrGridMap->data[index] = 0;
                }
            }
        }
        dCurrNum++;
        if((int)dCurrNum % (int)(dTotalSize/100) == 0)
        {
            this->publish(80 + 20*(dCurrNum/dTotalSize));
            if(_mbStopFlag)return false;
        }
    }
    _mptrGridMap->header.stamp = ros::Time::now();
    this->publish(99);

    std::string sMapSaveDirPath = THIS_PACKAGE_PATH + "/map_data";
    saveGridMap(sMapSaveDirPath);

    this->publish(100);

    return true;
}

void MapConverter::saveGridMap(std::string SaveDir)
{
    if(_mbStopFlag)return;

    std::ostringstream oss;
    oss << _mdReferZ;
    std::string strZ = oss.str();
    for (char& c : strZ){if (c == '.'){c = '_';}}

    // File name
    std::string sSavePGMPath = SaveDir + "/grid_map_" + strZ + ".pgm";
    std::string sSaveYAMLPath = SaveDir + "/grid_map_" + strZ + ".yaml";

    // Save Image (.pgm)
    std::ofstream ofsPGM(sSavePGMPath, std::ios::out);
    if (!ofsPGM.is_open()) {
        ROS_ERROR("Failed to open file: %s", sSavePGMPath.c_str());
        return;
    }
    ofsPGM << "P2\n";  // ASCII PGM (P2)
    ofsPGM << _mptrGridMap->info.width << " " << _mptrGridMap->info.height << "\n";
    ofsPGM << "255\n";  // max grayscale value
    for (int y = _mptrGridMap->info.height - 1; y >= 0; y--) {
        for (int x = 0; x < _mptrGridMap->info.width; x++) {
            int nTmpIdx = y * _mptrGridMap->info.width + x;
            int nTmpValue = _mptrGridMap->data[nTmpIdx];
            if (nTmpValue == -1) {
                ofsPGM << "205 ";  // unknown(gray, 205)
            } else if (nTmpValue == 100) {
                ofsPGM << "0 ";    // occupied(black, 0)
            } else {
                ofsPGM << "255 ";  // empty(white, 255)
            }
        }
        ofsPGM << "\n";
    }
    ofsPGM.close();

    if(_mbStopFlag)return;

    // Save YAML
    std::ofstream ofsYAML(sSaveYAMLPath , std::ios::out);
    if (!ofsYAML.is_open()) {
        ROS_ERROR("Failed to open file: %s", sSaveYAMLPath.c_str());
        return;
    }

    ofsYAML << "image: " << sSavePGMPath << "\n";
    ofsYAML << "resolution: " << _mptrGridMap->info.resolution << "\n";
    ofsYAML << "origin: [" << _mptrGridMap->info.origin.position.x << ", " << _mptrGridMap->info.origin.position.y << ", 0.0]\n";
    ofsYAML << "negate: 0\n";
    ofsYAML << "occupied_thresh: 0.65\n";  // occupancy thresold (0~1)
    ofsYAML << "free_thresh: 0.196\n";     // empty threshold (0~1)
    ofsYAML << "mode: trinary\n";          // thress mode(occupied, empty, unknown)
    ofsYAML.close();
}