#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include "DDPFF.h"

using namespace std;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "runFF");
    // Load and setup
    Config config;
    CloudBuffer pointcloudbuffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(config.pointCloud_fileName, *pointcloud) == -1) 
    {
        cout<<"Couldn't read file"<<endl;
        return 1;
    }
    else
    {
        std::cout << "Loaded:" << pointcloud->width<<"x"<<pointcloud->height<<" data points"<< std::endl;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);



///////////////////////////////////////////////////////////////////////////// DDPFF part////////////////////////////////////////////////////////////////////////////////////////////////////
        loadPointCloud(pointcloud, pointcloudbuffer);
        // DDPFF ddpff(config);
        // ddpff.init();
        // ddpff.setBuffer(&pointcloudbuffer);
        // ddpff.compute();

        // // for visualization purpose
        // std::vector< PlanePointNormal > plane_list = ddpff.getPlanes();
        // size_t plan_number = plane_list.size();
        // for (int i = 0; i < plan_number; i++) 
        // {
        //     // generate random color
        //     uint8_t r,g,b;
        //     srand(i);
        //     r = rand()%255;
        //     srand(i + plan_number);
        //     g = rand()%255;
        //     srand(i + plan_number + plan_number);
        //     b = rand()%255;
            
        //     for (size_t inlier : plane_list[i].inliers) 
        //     {
        //         pcl::PointXYZRGB point(r,g,b);
        //         point.x = pointcloudbuffer[inlier].x();
        //         point.y = pointcloudbuffer[inlier].y();
        //         point.z = pointcloudbuffer[inlier].z();
        //         pointcloud_colored->points.push_back(point);
        //     }
        // }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////// Region growing////////////////////////////////////////////////////////////////////////////////////////////////////
        pcl::search::Search< PointType >::Ptr  tree(new pcl::search::KdTree< PointType >());
        // Normal compute
        pcl::PointCloud< pcl::Normal >::Ptr normals(new pcl::PointCloud< pcl::Normal >());
        pcl::NormalEstimation< PointType, pcl::Normal > normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(pointcloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);
        cout << "Normal computed" << endl;

        // RegionGrowing
        pcl::RegionGrowing< PointType, pcl::Normal > reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(pointcloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1.0);
        
        //
        vector< pcl::PointIndices > clusters;
        reg.extract(clusters);
        cout << "Number of clusters is equal to " << clusters.size() << endl;
        pointcloud_colored = reg.getColoredCloud();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Visualization part
    pcl::visualization::CloudViewer viewer("Cloud view");  
    viewer.showCloud(pointcloud_colored);
    while(!viewer.wasStopped())
    {
        // keep viewing
    }



    // // ROS part
    
    // ros::NodeHandle nh;
    // ros::Publisher resultPublisher;
    // std_msgs::Header cloud_header;
    // resultPublisher = nh.advertise<sensor_msgs::PointCloud2> ("FF", 1);


    // std::vector< PlanePointNormal >                  plane_list = ddpff.getPlanes();
    // for (int i = 0; i < plane_list.size(); i++) 
    // {
    //     for (size_t inlier : plane_list[i].inliers) {
    //         PointType point;+ 
    //         // ceiling_cloud->push_back(pointcloudbuffer[inlier]);
    //     }
    // }


    // ros::Rate rate(10);
    // while (ros::ok()) {
    //     sensor_msgs::PointCloud2 tempCloud;
    //     tempCloud.header.stamp = ros::Time::now();
    //     pcl::toROSMsg(*pointcloud, tempCloud);
    //     tempCloud.header.frame_id = "temp";
    //     resultPublisher.publish(tempCloud);
    //     rate.sleep();
    // }
    // ros::spin();
    return 0;
}