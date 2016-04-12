#ifndef CAR_MARKER_H
#define CAR_MARKER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


#include <string.h>
#include <vector>

class CarMarker {

public:
    CarMarker() : marker_seq(0) {}

    static float colormap[24][3];

    std::vector<visualization_msgs::Marker> createCar(
            int id,
            geometry_msgs::Pose pose,
            std::string target_frame) {
        std::vector<visualization_msgs::Marker> car(1);
        car[0].header.frame_id = target_frame;
        car[0].header.stamp = ros::Time::now();
        car[0].header.seq = ++marker_seq;
        car[0].ns = "people_tracker";
        car[0].id = id;
        car[0].lifetime = ros::Duration(1.0);
        car[0].frame_locked = true;

        //car[0].pose.position.x = 2.4 + -1.3;
        //car[0].pose.position.z = 0.75;
        //car[0].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2.0);

        car[0].type = visualization_msgs::Marker::MESH_RESOURCE;
        car[0].action = visualization_msgs::Marker::ADD;
        car[0].pose = pose;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.orientation, quat);
        quat *= tf::createQuaternionFromYaw(M_PI/2.0);
        tf::quaternionTFToMsg(quat, car[0].pose.orientation);

        // mesh vehicle proportions are about right, but its size is very small
        car[0].scale.x = 0.024;
        car[0].scale.y = 0.024;
        car[0].scale.z = 0.024;
        car[0].color.r = colormap[id%24][0]/255.0;
        car[0].color.g = colormap[id%24][1]/255.0;
        car[0].color.b = colormap[id%24][2]/255.0;
        car[0].color.a = 0.8;
        car[0].mesh_resource = "package://bayes_people_tracker/etc/car.stl";
        car[0].mesh_use_embedded_materials = false;

        return car;
    }

private:

    unsigned long marker_seq;

};

float CarMarker::colormap[24][3] = {
    {166,206,227},
    {31,120,180},
    {178,223,138},
    {51,160,44},
    {251,154,153},
    {227,26,28},
    {253,191,111},
    {255,127,0},
    {202,178,214},
    {106,61,154},
    {255,255,153},
    {177,89,40},
    {141,211,199},
    {255,255,179},
    {190,186,218},
    {251,128,114},
    {128,177,211},
    {253,180,98},
    {179,222,105},
    {252,205,229},
    {217,217,217},
    {188,128,189},
    {204,235,197},
    {255,237,111}
};


#endif // CAR_MARKER_H
