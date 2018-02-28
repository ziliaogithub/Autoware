# Multi LiDAR Calibrator

This package allows to obtain the extrinsic calibration between two PointClouds with the help of the NDT algorithm.


The `multi_lidar_calibrator` node receives two `PointCloud2` messages (parent and child), and an initialization pose from rviz.
The transformation required to transform the child to the parent point cloud is calculated, if possible, output to the terminal and registered in the TF tree.

## How to launch

In a sourced terminal:

`roslaunch multi_lidar_calibrator multi_lidar_calibrator points_src_parent:=/lidarA/points_raw points_src_child:=/lidarB/points_raw`

## Input topics

|Parameter| Type| Description|
----------|-----|--------
|`point_src_parent`|*String* |PointCloud topic name to subscribe and synchronize with the child.|
|`point_src_child`|*String*|PointCloud topic name to subscribe and synchronize with the parent.|

## Output

1. TF registration between the two pointcloud frames
1. Output in the terminal showing the X,Y,Z,R,P,Y transformation between child and parent. These values can be used later with the `static_transform_publisher`.