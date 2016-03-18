#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class publish_cloud_server {

private:

    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;

    ros::NodeHandle n;
    ros::Publisher pub;

    boost::filesystem::path path;

public:

    void run()
    {
        for (int i = 0; ; ++i) {
            std::stringstream ss;
            ss << std::setw(10) << std::setfill('0') << i;
            boost::filesystem::path file = path / (ss.str() + ".pcd");
            if (!boost::filesystem::exists(file)) {
                break;
            }
            CloudT::Ptr cloud(new CloudT);
            pcl::io::loadPCDFile(file.string(), *cloud);
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.frame_id = "/map";
            cloud_msg.header.stamp = ros::Time::now();
            pub.publish(cloud_msg);
            //ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    publish_cloud_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        string folder;
        pn.param<string>("folder", folder, "");
        path = boost::filesystem::path(folder);

        pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne", 1);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_velodyne_points");

    publish_cloud_server pcs(ros::this_node::getName());

    pcs.run();

    return 0;
}
