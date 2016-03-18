#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class elevation_map_server {

private:

    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    double grid_size;
    double max_radius;
    double double_radius;
    int width;
    int height;

public:

    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        CloudT::Ptr cloud(new CloudT);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        ROS_INFO("Converted point cloud");

        cv::Mat image = std::numeric_limits<float>::infinity()*cv::Mat::ones(height, width, CV_32FC1);

        for (const PointT& p : cloud->points) {
            // first discretize into grid cell, and then assign elevation to this index
            // we're assuming that the sensor is at (0, 0)
            float dist = sqrt(p.x*p.x+p.y*p.y);

            int x = int((p.x - (-max_radius)) / grid_size);
            int y = int((p.y - (-max_radius)) / grid_size);

            int neighborhood = int(dist/double_radius);

            for (int i = y-neighborhood; i <= y+neighborhood; ++i) {
                if (i < 0 || i >= height) {
                    continue;
                }
                for (int j = x-neighborhood; j <= x+neighborhood; ++j) {
                    if (j < 0 || j >= width) {
                        continue;
                    }
                    float& height = image.at<float>(i, j);
                    if (std::isfinite(height)) {
                        height = std::max(height, p.z);
                    }
                    else {
                        height = p.z;
                    }
                }
            }
        }

        ROS_INFO("Created image");

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->encoding = "32FC1";
        cv_pub_ptr->image = image;
        //cv_pub_ptr->encoding = "bgr8";
        sensor_msgs::Image ros_image = *cv_pub_ptr->toImageMsg();
        pub.publish(ros_image);

        ROS_INFO("Published image");
    }

    elevation_map_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<double>("grid_size", grid_size, 0.2);
        pn.param<double>("max_radius", max_radius, 20.0);
        pn.param<double>("double_radius", double_radius, 7.0);

        width = height = int(2*max_radius/grid_size);

        pub = n.advertise<sensor_msgs::Image>("/elevation", 1);
        sub = n.subscribe("/velodyne", 1, &elevation_map_server::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_elevation_map");

    elevation_map_server ems(ros::this_node::getName());

    ros::spin();

    return 0;
}
