#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <oxts/oxts.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

class publish_cloud_transform_server {

private:


    // one thing to hold the preloaded transforms with time stamps
    vector<ros::Time> transform_stamps;
    vector<tf::Transform> transforms;

    // and also the preloaded timestamps of the velodyne
    vector<ros::Time> velodyne_stamps;

    tf::TransformBroadcaster br;

    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;

    ros::NodeHandle n;
    ros::Publisher pub;

    boost::filesystem::path cloud_path;
    boost::filesystem::path cloud_stamps_path;
    boost::filesystem::path transforms_path;
    boost::filesystem::path transforms_stamps_path;

public:

    bool timestamp_to_nsec(string timestamp, unsigned long &time){
        struct tm tm;
        if (strptime(timestamp.c_str(), "%Y-%m-%d %H:%M:%S", &tm) != NULL ){
            unsigned long sec = (unsigned long)(mktime(&tm)*1000000000);
            unsigned long milli = (unsigned long)(std::atof(("0." +
                        timestamp.substr(timestamp.find_last_of(".\\")+1)).c_str())*1000000000);
            time = sec+milli;
            return true;
        }
        return false;
    }

    void load_timestamps(vector<ros::Time>& timestamps, const boost::filesystem::path& timepath)
    {
        ifstream f(timepath.string());
        string line;
        while (getline(f, line)) {
            unsigned long time;
            if (!timestamp_to_nsec(line, time)) {
                break;
            }
            ros::Time ros_time;
            ros_time.fromNSec(time);
            timestamps.push_back(ros_time);
        }
    }

    double lat_to_scale(const double lat){
        return cos(lat * M_PI / 180.0);
    }

    void latlon_to_mercator(const double lat, const double lon, const double scale, double &mx, double &my){
        double er = 6378137;
        mx = scale * lon * M_PI * er / 180;
        my = scale * er * log( tan((90+lat) * M_PI / 360) );
    }

    void load_transforms(vector<tf::Transform>& transforms, const boost::filesystem::path& path)
    {
        string line;
        bool first = true;
        Eigen::Affine3d first_inverse;
        double scale;
        for (int i = 0; ; ++i) {
            std::stringstream ss;
            ss << std::setw(10) << std::setfill('0') << i;
            boost::filesystem::path file = path / (ss.str() + ".txt");
            if (!boost::filesystem::exists(file)) {
                break;
            }
            ifstream f(file.string());
            if (!getline(f, line)) {
                cout << "Could not open transform file " << file.string() << endl;
                exit(0);
            }

            std::istringstream iss(line);
            vector<double> vec;
            std::copy(std::istream_iterator<double>(iss),
                      std::istream_iterator<double>(),
                      std::back_inserter(vec));
            if (first) {
                scale = lat_to_scale(vec[0]);
            }

            // Create the transformation matrix.
            // Matrix4f transform = Matrix4f::Identity();
            Eigen::Affine3d transform = Eigen::Affine3d::Identity();

            // Fill in translation part.
            double mx, my;
            latlon_to_mercator(vec[0], vec[1], scale, mx, my);
            transform.translation() << mx, my, 0;

            // Add rotation part.
            transform.rotate(Eigen::AngleAxisd(vec[3], Eigen::Vector3d::UnitX()));
            transform.rotate(Eigen::AngleAxisd(vec[4], Eigen::Vector3d::UnitY()));
            transform.rotate(Eigen::AngleAxisd(vec[5], Eigen::Vector3d::UnitZ()));

            if (first) {
                // normalize translation and rotation (start at 0/0/0)
                first_inverse = transform.inverse();
            }

            first = false;

            tf::Transform tf_transform;
            tf::transformEigenToTF(first_inverse*transform, tf_transform);
            transforms.push_back(tf_transform);
        }

    }

    void run()
    {
        ros::Time start_time = ros::Time::now();
        ros::Time start_sim_time = transform_stamps[0];

        int transform_index = 0;
        int cloud_index = 0;
        while (cloud_index < velodyne_stamps.size() && transform_index < transforms.size()) {

            ros::Duration elapsed = ros::Time::now() - start_time;
            ros::Time sim_time = start_sim_time + elapsed;

            if (velodyne_stamps[cloud_index] - sim_time < ros::Duration(0.02)) {
                std::stringstream ss;
                ss << std::setw(10) << std::setfill('0') << cloud_index;
                boost::filesystem::path file = cloud_path / (ss.str() + ".pcd");
                if (!boost::filesystem::exists(file)) {
                    break;
                }
                CloudT::Ptr cloud(new CloudT);
                pcl::io::loadPCDFile(file.string(), *cloud);
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);
                cloud_msg.header.frame_id = "/velodyne";
                cloud_msg.header.stamp = ros::Time::now();
                pub.publish(cloud_msg);
                elapsed = ros::Time::now() - start_time;
                sim_time = start_sim_time + elapsed;

                ++cloud_index;
            }

            ros::Duration sleep_time(transform_stamps[transform_index]-sim_time);
            cout << "Sleeping for " << sleep_time.toSec() << " secs" << endl;
            if (sleep_time.toSec() > 0) {
                sleep_time.sleep();
            }
            br.sendTransform(tf::StampedTransform(transforms[transform_index], ros::Time::now(), "map", "velodyne"));
            ++transform_index;
        }
    }

    publish_cloud_transform_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        string cloud_folder;
        pn.param<string>("cloud_path", cloud_folder, "");
        cloud_path = boost::filesystem::path(cloud_folder);

        string cloud_stamps;
        pn.param<string>("cloud_stamps_path", cloud_stamps, "");
        cloud_stamps_path = boost::filesystem::path(cloud_stamps);

        string transforms_file;
        pn.param<string>("transforms_path", transforms_file, "");
        transforms_path = boost::filesystem::path(transforms_file);

        string transforms_stamps;
        pn.param<string>("transforms_stamps_path", transforms_stamps, "");
        transforms_stamps_path = boost::filesystem::path(transforms_stamps);

        load_timestamps(velodyne_stamps, cloud_stamps_path);
        load_timestamps(transform_stamps, transforms_stamps_path);
        load_transforms(transforms, transforms_path);

        cout << "Loaded " << transforms.size() << " transforms" << endl;
        cout << "Loaded " << transform_stamps.size() << " transform stamps" << endl;
        cout << "Loaded " << velodyne_stamps.size() << " cloud stamps" << endl;

        pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne", 1);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_velodyne_transforms");

    publish_cloud_transform_server pcs(ros::this_node::getName());

    pcs.run();

    return 0;
}
