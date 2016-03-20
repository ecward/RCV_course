Velodyne Tracking
=================

This package does detection and tracking of dynamic sections
of velodyne point clouds. It achieves this by projecting
the point cloud onto an elevation map in the xy-plane.
Potential obstacles are detected in the elevation map using
a simple OpenCV blob detector. The detected blobs are then
tracked using an EKF based filter.

To run simply execute the launch file
```
roslaunch velodyne_tracking tracking.launch
```
For available parameters, see this launch file together
with `detection.launch` and `tracking.launch`.
To try with some example point clouds from the KITTI dataset,
simply run
```
rosrun velodyne_tracking publish_velodyne_clouds _folder:=/path/to/data
```
to continuously publish point clouds in the folder at 10 hz.

# Marker Visualizations

In the package `bayes_people_tracker` there are utilities for
visualizing humans and cars via marker types in Rviz. The
relevant header files are located in
`bayes_people_tracker/include/bayes_people_tracker/people_marker.h`
and
`bayes_people_tracker/include/bayes_people_tracker/car_marker.h`.
For and example of how to use them, see
`bayes_people_tracker/src/bayes_people_tracker/people_tracker.cpp`.

# KITTI Dataset

In `velodyne_tracking` there are utilities for loading point
clouds, transforms and timestamps from the KITTI dataset and
simulating one of the runs in a ROS friendly way with TF etc.
You can run this with the following command:
```
roslaunch velodyne_tracking kitti_simulation.launch folder:=/path/to/data
```
For details, check the launch file. This will probably
be split up into its own package at some point.
