# FloodFill algorithm on LiDAR PointCloud
A plane segmentation algorithm for lidar point clouds, which is part of [FP-Loc 2.0](https://fplocextension.github.io). This repository contains code for running plane segmentation on single-frame lidar point clouds with ROS (Robot Operating System).

As the name of the algorithm suggests, this plane segmentation algorithm is based on flood fill, with optimizations for lidar, making it able to quickly and accurately segment out the planes in the lidar point cloud, and meet the needs of fields like robotics (such as SLAM framework). 

## Build and Run
We use [ROS](https://www.ros.org/) to load configs, which contains modifiable parameters. After performing the plane segmentation algorithm, we used [PCL](https://pointclouds.org/) to assist with visualization. Below are the demo codes that can be used for Plane Segmentation Visualization on a single `.pcd` file.

``` bash
cd ~/catkin_ws/src
git clone https://github.com/ChunhBi/FloodFill_For_LiDAR.git
cd .. && catkin_make
source /devel/setup.sh
roslaunch lidar_floodfill runFF.launch
```
The input file path and other FloodFill parameters are in `configs/default.yaml` in default. The config file can be changed in `launch/runFF.launch`.

We tested that this code can run normally on Ubuntu 18 and Ubuntu 20 systems. 

##   Acknowledgement：
Our code borrows a lot from [DDPFF](https://github.com/arindamrc/DDPFF).


##   TODO：
Publish ROS msg that shows plane segmentation results using different colors.
Current visualization uses pcl library.
