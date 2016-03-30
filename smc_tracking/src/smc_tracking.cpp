#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <smc_tracking/smc_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;

class smc_tracking_server {

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher particle_pub_1;
    ros::Publisher particle_pub_2;
    tf::TransformListener listener;

    double grid_size;
    double max_radius;
    double likelihood_threshold;

    ros::Time last_time;

    vector<smc_filter> filters;
    Eigen::VectorXi steps_since_last;

public:

    geometry_msgs::PoseArray create_pose_array(const vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& estimates)
    {
        geometry_msgs::PoseArray tracked_poses;
        for (const Eigen::Vector4f& e : estimates) {
            geometry_msgs::Pose pose;
            pose.position.x = e(0);
            pose.position.y = e(1);
            pose.position.z = 0.0f;

            Eigen::AngleAxisf angle_axis(e(2), Eigen::Vector3f::UnitZ());
            Eigen::Quaternionf quat(angle_axis);
            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();

            tracked_poses.poses.push_back(pose);
        }
        return tracked_poses;
    }

    void callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        // how do we associate the new measurements with a filter?
        // or I guess we don't have to, but then put them in a kdtree?
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/velodyne", ros::Time(0), transform); // msg->header.stamp
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        float timestep = (msg->header.stamp - last_time).toSec();
        last_time = msg->header.stamp;

        int N = msg->poses.size();
        Eigen::Matrix<float, Eigen::Dynamic, 2> detections(N, 2);

        int counter = 0;
        for (int i = 0; i < msg->poses.size(); ++i) {
            tf::Point position;
            tf::pointMsgToTF(msg->poses[i].position, position);
            position = transform*position;
            if (std::isnan(position.x()) || std::isnan(position.y())) {
                --N;
                continue;
            }
            detections(counter, 0) = position.x();
            detections(counter, 1) = position.y();
            ++counter;
        }
        detections.conservativeResize(N, 2);

        smc_filter::kdtree mat_index(2, detections, 10);
        mat_index.index->buildIndex();

        // check if there are any filters corresponding to the detections,
        // otherwise, intialize a new filter. Probably if there are no new
        // evidence for a filter in the last few timesteps, remove filter
        vector<smc_filter> new_filters;
        steps_since_last.array() += 1;
        for (int i = 0; i < N; ++i) {
            bool filter_found = false;
            float best_likelihood = 0.0f;
            int best_index;
            for (int j = 0; j < filters.size(); ++j) {
                float likelihood = filters[j].likelihood(detections.row(i).transpose());
                if (likelihood > likelihood_threshold) {
                    filter_found = true;
                    if (likelihood > best_likelihood) {
                        best_index = j;
                        best_likelihood = likelihood;
                    }
                    //break;
                }
            }
            if (filter_found) {
                steps_since_last(best_likelihood) = 0;
            }
            else {
                for (int j = 0; j < new_filters.size(); ++j) {
                    float likelihood = new_filters[j].likelihood(detections.row(i).transpose());
                    if (likelihood > likelihood_threshold) {
                        filter_found = true;
                        break;
                    }
                }
            }
            if (!filter_found) {
                smc_filter filter;
                filter.init_with_observation(detections.row(i).transpose());
                new_filters.push_back(filter);
            }
        }

        vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > estimates;
        counter = 0;
        for (smc_filter& filter : filters) {
            cout << "Filter: " << counter << endl;
            filter.propagate(timestep);
            filter.resample(mat_index);
            Eigen::Vector4f estimate = filter.estimate();
            estimates.push_back(estimate);
            ++counter;
        }
        steps_since_last.conservativeResize(filters.size() + new_filters.size());
        steps_since_last.tail(new_filters.size()).setZero();
        filters.insert(filters.end(), new_filters.begin(), new_filters.end());

        for (int i = filters.size() - 1; i >= 0; --i) {
            if (steps_since_last(i) > 10) {
                cout << i << endl;
                cout << filters.size() << endl;
                cout << steps_since_last.rows() << endl;
                filters.erase(filters.begin() + i);
                if (i < filters.size() - 1) {
                    steps_since_last.segment(i, filters.size()-i-1) = steps_since_last.segment(i+1, filters.size()-i-1);
                }
                steps_since_last.conservativeResize(filters.size());
            }
        }

        geometry_msgs::PoseArray tracked_poses = create_pose_array(estimates);

        tracked_poses.header.frame_id = "/map";
        tracked_poses.header.stamp = msg->header.stamp;
        pub.publish(tracked_poses);

        if (filters.size() > 2) {
            geometry_msgs::PoseArray particles_1 = create_pose_array(filters[0].get_particles());
            geometry_msgs::PoseArray particles_2 = create_pose_array(filters[1].get_particles());

            particles_1.header.frame_id = "/map";
            particles_1.header.stamp = msg->header.stamp;
            particle_pub_1.publish(particles_1);

            particles_2.header.frame_id = "/map";
            particles_2.header.stamp = msg->header.stamp;
            particle_pub_2.publish(particles_2);
        }
    }

    smc_tracking_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<double>("grid_size", grid_size, 0.2);
        pn.param<double>("max_radius", max_radius, 20.0);
        pn.param<double>("likelihood_threshold", likelihood_threshold, 0.001);

        pub = n.advertise<geometry_msgs::PoseArray>("/tracked_blobs", 1);
        particle_pub_1 = n.advertise<geometry_msgs::PoseArray>("/particles_1", 1);
        particle_pub_2 = n.advertise<geometry_msgs::PoseArray>("/particles_2", 1);
        sub = n.subscribe("/velodyne_detections", 1, &smc_tracking_server::callback, this);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smc_tracking");

    smc_tracking_server smcs(ros::this_node::getName());

    ros::spin();

    return 0;
}
