#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

using namespace std;

class edge_map_server {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

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

        cv::Mat detected_edges;

        //int edgeThresh = 1;
        int lowThreshold = 50;
        int ratio = 3;
        int kernel_size = 3;

        /// Canny detector
        cv::Canny(cv_ptr->image, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

        cv::Mat mask = cv_ptr->image == 0;
        detected_edges.setTo(cv::Scalar(0), mask);

        /// Using Canny's output as a mask, we display our result
        //cv::Mat dst = Scalar::all(0);

        //src.copyTo(dst, detected_edges);

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->encoding = "mono8";
        cv_pub_ptr->image = detected_edges;
        sensor_msgs::Image ros_image = *cv_pub_ptr->toImageMsg();
        pub.publish(ros_image);

        ROS_INFO("Published image");
    }

    edge_map_server(const std::string& name)
    {
//        ros::NodeHandle pn("~");
//        pn.param<double>("max_height", max_height, 1.0);
//        pn.param<double>("min_height", min_height, -1.5);

        pub = n.advertise<sensor_msgs::Image>("/edges_elevation", 1);
        sub = n.subscribe("/filled_elevation", 1, &edge_map_server::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_edge_map");

    edge_map_server ems(ros::this_node::getName());

    ros::spin();

    return 0;
}
