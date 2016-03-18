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
to continuously publish point clouds in the folder at 1 hz.
