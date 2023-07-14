project(lidar_floodfill)
cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy

  # pcl library
  pcl_ros
  pcl_conversions
  
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
	${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_runFF 
src/Config.cpp src/DDPFF.cpp src/runFF.cpp  )

target_link_libraries(${PROJECT_NAME}_runFF 
${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})