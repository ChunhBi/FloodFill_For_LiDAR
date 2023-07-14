#include "Config.h"

Config::Config() 
{
    nh.param<std::string>("FF/pointCloud_fileName", pointCloud_fileName, " ./src/data/school_scooter/14.pcd");
    nh.param<int>("FF/IMAGE_WIDTH", IMAGE_WIDTH, 2048);
    nh.param<int>("FF/IMAGE_HEIGHT", IMAGE_HEIGHT, 128);

    nh.param<double>("FF/bufferSize", bufferSize, 30);
    nh.param<int>("FF/sampleFactor", sampleFactor, 1);

    // IMPORTANT: point distance part
    nh.param<double>("FF/pointThresholdFloodFill_min", pointThresholdFloodFill_min, 0.001);
    nh.param<double>("FF/pointThresholdFloodFill_max", pointThresholdFloodFill_max, 0.1);
    nh.param<double>("FF/c_point", c_point, 0.1);

    // IMPORTANT: plane size part
    nh.param<double>("FF/normalSampleDistance_min", normalSampleDistance_min, 4.0f);
    nh.param<double>("FF/normalSampleDistance_max", normalSampleDistance_max, 20.0f);
    nh.param<double>("FF/c_range", c_range, 1.0);

    // planeThresholdFloodFill_flood = 0.01;
    nh.param<double>("FF/planeThresholdFloodFill_flood", planeThresholdFloodFill_flood, 0.0001);
    nh.param<double>("FF/planeThresholdFloodFill_merge", planeThresholdFloodFill_merge, 0.01);
    nh.param<double>("FF/planeThresholdFloodFill_flood_max", planeThresholdFloodFill_flood_max, 0.01);
    nh.param<double>("FF/planeThresholdFloodFill_merge_max", planeThresholdFloodFill_merge_max, 0.1);
    nh.param<double>("FF/angleThresholdFloodFill", angleThresholdFloodFill, 0.05);
    nh.param<double>("FF/angleThresholdFloodFill_max", angleThresholdFloodFill_max, 0.2);
    nh.param<double>("FF/minPlaneSize", minPlaneSize, 1000);
    nh.param<double>("FF/c_plane", c_plane, 1.5);
    nh.param<double>("FF/c_plane_merge", c_plane_merge, 8);
    nh.param<double>("FF/c_angle", c_angle, 2);
}



void loadPointCloud(const pcl::PointCloud< PointType >::Ptr cloud, CloudBuffer & pointBuffer) 
{
    for (auto& point : cloud->points) 
    {
        pointBuffer.push_back(Vec3(point.x,point.y,point.z));
    }
    std::cout<<"loaded pointcloud"<<std::endl;
}