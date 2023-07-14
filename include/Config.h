#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <array>
#include <string>
#include <ros/ros.h>
#include "constants.h"
#include "utilities.h"

class Config 
{
public:
    Config();
    ~Config() {}

    ros::NodeHandle nh;
    std::string pointCloud_fileName;
    int IMAGE_WIDTH;
    int IMAGE_HEIGHT;

    double bufferSize;
    int sampleFactor;

    // flood fill parameters
    double pointThresholdFloodFill_min;
    double pointThresholdFloodFill_max;
    double planeThresholdFloodFill_flood;
    double planeThresholdFloodFill_merge;
    double planeThresholdFloodFill_flood_max;
    double planeThresholdFloodFill_merge_max;
    double angleThresholdFloodFill;
    double angleThresholdFloodFill_max;
    double normalSampleDistance_min;
    double normalSampleDistance_max;
    double c_plane;
    double c_plane_merge;
    double c_point;
    double c_angle;
    double c_range;

    // common for flood fill and ransac
    double minPlaneSize;
};

// Load the lidar pointcloud
void loadPointCloud(const pcl::PointCloud< PointType >::Ptr cloud, CloudBuffer& pointBuffer);

#endif // CONFIG_H
