#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "contour_blob_detector/contour_blob_detector.h"

using namespace std;

class blob_server {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher shape_pub;
    ros::Publisher detections_pub;
    //tf::TransformListener listener;

    double grid_size;
    double max_radius;

public:

    void publish_marker_message(const vector<vector<cv::Point> >& contours)
    {
        visualization_msgs::MarkerArray markers;
        int marker_id = 0;
        for (const vector<cv::Point>& contour : contours) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "/velodyne";
            marker.header.stamp = ros::Time::now();
            marker.ns = "shapes";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker.scale.x = 0.1f;
            for (const cv::Point& p : contour) {
                geometry_msgs::Point q;
                q.x = grid_size*p.x - max_radius;
                q.y = grid_size*p.y - max_radius;
                q.z = -1.8f;
                marker.points.push_back(q);
            }
            if (!marker.points.empty()) {
                marker.points.push_back(marker.points.front());
            }

            markers.markers.push_back(marker);
        }
        shape_pub.publish(markers);
    }

    void callback(const sensor_msgs::Image::ConstPtr& image_msg)
    {
        /*
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        */

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

        // Change thresholds
        params.minDistBetweenBlobs = 5.0f;

        // Various filters
        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.2;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 30.0f;
        //params.maxArea = 200.0f;
        params.maxArea = 100.0f;

        // Set up the detector with default parameters.
        //cv::SimpleBlobDetector detector(params);
        contour_blob_detector detector(params);

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(cv_ptr->image, keypoints);

        vector<vector<cv::Point> > contours = detector.getContours();
        contours.resize(keypoints.size());
        publish_marker_message(contours);
        //keypoints.resize(3);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat im_with_keypoints;
        drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawContours(im_with_keypoints, contours, -1, cv::Scalar(255,0,0), 1, 8);

        geometry_msgs::PoseArray detections_msg;
        detections_msg.header.frame_id = "/velodyne";
        detections_msg.header.stamp = ros::Time::now();
        for(const cv::KeyPoint& k : keypoints){
           std::cout << "size of blob is: " << k.size << std::endl;
           std::cout << "point is at: " << k.pt.x << " " << k.pt.y << std::endl;

           //int((p.x - (-max_radius)) / grid_size);
           geometry_msgs::Pose pose_msg;
           pose_msg.position.x = grid_size*k.pt.x - max_radius; // + transform.getOrigin().x()
           pose_msg.position.y = grid_size*k.pt.y - max_radius; // + transform.getOrigin().y()
           pose_msg.position.z = 0.0f;

           pose_msg.orientation.x = 0.0f;
           pose_msg.orientation.y = 0.0f;
           pose_msg.orientation.z = 1.0f;
           pose_msg.orientation.w = 0.0f;

           detections_msg.poses.push_back(pose_msg);
        }
        detections_pub.publish(detections_msg);

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->encoding = "rgb8";
        cv_pub_ptr->image = im_with_keypoints;
        sensor_msgs::Image ros_image = *cv_pub_ptr->toImageMsg();
        pub.publish(ros_image);

        ROS_INFO("Published image");
    }

    blob_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<double>("grid_size", grid_size, 0.2);
        pn.param<double>("max_radius", max_radius, 20.0);

        /*
        ROS_INFO("Waiting for transform from /velodyne to /map...");
        try {
            listener.waitForTransform("/map", "/velodyne", ros::Time(0), ros::DURATION_MAX);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        */

        pub = n.advertise<sensor_msgs::Image>("/velodyne_blobs", 1);
        shape_pub = n.advertise<visualization_msgs::MarkerArray>("/velodyne_outlines", 1);
        detections_pub = n.advertise<geometry_msgs::PoseArray>("/velodyne_detections", 1);
        sub = n.subscribe("/filled_elevation", 1, &blob_server::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_elevation_blobs");

    blob_server bs(ros::this_node::getName());

    ros::spin();

    return 0;
}

