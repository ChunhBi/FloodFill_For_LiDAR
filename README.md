# FloodFill algorithm on LiDAR PointCloud

The codes are modified from the public project [DDPFF](https://github.com/arindamrc/DDPFF), which is mainly used and tested on Depth Camera point cloud. We pruned the codes to be more simple and lightweight, which suits for our project [FP-Loc 2.0](https://fplocextension.github.io). This repository contains codes that do FloodFill Plane Segmentation on single LiDAR point cloud.

## Build and Run
We use ROS to load configs, which contains modifiable parameters. Below are demo codes that can be used for Plane Segmentation Visualization on a single .pcd file.

``` bash
catkin_make
./devel/setup.bash
roslaunch lidar_floodfill runFF.launch
```
The input file name and other FloodFill parameters are in configs/default.yaml in default. The config file to use can be changed in launch/runFF.launch.



##   TODO：
Publish ROS msg that shows plane segmentation results using different colors.
Current visualization uses pcl library.
