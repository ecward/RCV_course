#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

class project_velodyne_node {
public:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::TransformListener listener;
    Eigen::Matrix4f eigen_transform;
    string sensor_frame;

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        pcl::transformPointCloud(*cloud, *cloud, eigen_transform);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.3, 1.5);
        //pass.setFilterLimitsNegative (true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter (*cloud_filtered);
        for (size_t i = 0; i < cloud_filtered->size(); ++i) {
            cloud_filtered->points[i].z = 0.0f;
        }
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud_filtered, out_msg);
        out_msg.header.stamp = ros::Time::now(); //msg->header.stamp;
        out_msg.header.frame_id = "base_link";
        pub.publish(out_msg);
    }

    project_velodyne_node(const string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("sensor_frame", sensor_frame, std::string(""));

        pub = n.advertise<sensor_msgs::PointCloud2>("/output", 1);

        tf::StampedTransform tf_transform;
        listener.waitForTransform("/base_link", sensor_frame,ros::Time::now(), ros::Duration(3.0));
        listener.lookupTransform("/base_link", sensor_frame, ros::Time(0), tf_transform);
        pcl_ros::transformAsMatrix(tf_transform, eigen_transform);

        sub = n.subscribe("/input", 10, &project_velodyne_node::callback, this);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "project_velodyne");

    project_velodyne_node pvn(ros::this_node::getName());

    ros::spin();

    return 0;
}
