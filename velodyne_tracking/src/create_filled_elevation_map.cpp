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

class fill_map_server {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    double max_height;
    double min_height;

public:

    void callback(const sensor_msgs::Image::ConstPtr& image_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;

        cv::Mat discretized(height, width, CV_8UC1);
        float step = (max_height - min_height)/255;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float height = cv_ptr->image.at<float>(y, x);
                if (!std::isfinite(height) || std::isnan(height)) {
                    discretized.at<uchar>(y, x) = 0;
                    continue;
                }
                int discretized_height = (height - min_height)/step;
                if (discretized_height <= 0) {
                    discretized.at<uchar>(y, x) = 1;
                }
                else if (discretized_height > 255) {
                    discretized.at<uchar>(y, x) = 255;
                }
                else {
                    discretized.at<uchar>(y, x) = discretized_height;
                }
            }
        }

        cv::Mat mask = discretized != 0;

        cv::Mat filtered_mask;
        cv::erode(mask, filtered_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
        cv::dilate(filtered_mask, filtered_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
        cv::bitwise_not(filtered_mask, filtered_mask);
        discretized.setTo(cv::Scalar(0), filtered_mask);

        cv::Mat area_mask;
        cv::dilate(mask, area_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7)));

        cv::Mat needs_filling;
        cv::bitwise_and(discretized == 0, area_mask, needs_filling);
        cv::inpaint(discretized, needs_filling, discretized, 2, cv::INPAINT_TELEA);

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->encoding = "mono8";
        cv_pub_ptr->image = discretized;
        //cv_pub_ptr->encoding = "bgr8";
        sensor_msgs::Image ros_image = *cv_pub_ptr->toImageMsg();
        pub.publish(ros_image);

        ROS_INFO("Published image");
    }

    fill_map_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<double>("max_height", max_height, 1.0);
        pn.param<double>("min_height", min_height, -1.5);

        pub = n.advertise<sensor_msgs::Image>("/filled_elevation", 1);
        sub = n.subscribe("/elevation", 1, &fill_map_server::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_filled_elevation_map");

    fill_map_server fms(ros::this_node::getName());

    ros::spin();

    return 0;
}
