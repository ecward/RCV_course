#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

class blob_server {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher detections_pub;

    double grid_size;
    double max_radius;

public:

    void callback(const sensor_msgs::Image::ConstPtr& image_msg)
    {
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
        params.minDistBetweenBlobs = 20.0f;

        // Various filters
        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.1;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 20.0f;
        params.maxArea = 200.0f;

        // Set up the detector with default parameters.
        cv::SimpleBlobDetector detector(params);

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(cv_ptr->image, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat im_with_keypoints;
        drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        geometry_msgs::PoseArray detections_msg;
        detections_msg.header.frame_id = "/map";
        detections_msg.header.stamp = ros::Time::now();
        for(const cv::KeyPoint& k : keypoints){
           std::cout << "size of blob is: " << k.size << std::endl;
           std::cout << "point is at: " << k.pt.x << " " << k.pt.y << std::endl;

           //int((p.x - (-max_radius)) / grid_size);
           geometry_msgs::Pose pose_msg;
           pose_msg.position.x = grid_size*k.pt.x - max_radius;
           pose_msg.position.y = grid_size*k.pt.y - max_radius;
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

        pub = n.advertise<sensor_msgs::Image>("/edges_elevation", 1);
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

