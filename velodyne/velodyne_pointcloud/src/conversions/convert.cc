/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"
#include <time.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    private_nh.param("stream_id",stream_id_,std::string(""));

    offline_mode_ = private_nh.param("pcap", std::string("")) != "";

    ROS_INFO_STREAM("Starting velodyne converter id = " << stream_id_ << " running in " << (offline_mode_ ? "offline mode":"online mode"));

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points_"+stream_id_, 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      VelodyneConfigConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::VelodyneConfigConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    ROS_INFO_STREAM("Subscribing to " << "velodyne_packets_"+stream_id_);

    // subscribe to VelodyneScan packets    
    velodyne_scan_ =
      node.subscribe("velodyne_packets_"+stream_id_, 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));


  }


  
  void Convert::callback(velodyne_pointcloud::VelodyneConfigConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
      ROS_DEBUG("Convert::processScan()");

      if (output_.getNumSubscribers() == 0)         // no one listening?
          return;                                     // avoid much work

      // allocate a point cloud with same time and frame ID as raw data
      velodyne_rawdata::VPointCloud::Ptr
              outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsg->header.frame_id = scanMsg->header.frame_id;
      outMsg->height = 1;

      //figure out current hour and seconds since it
      //If we are running in normal mode we only need to look at the system
      //clock to figure out the actual timestamp
      //If we are running offline (e.g. using a pcap log) we need to get the current
      //hour from /clock which we get by setting
      // use_sim_time = true in the launch file

      ///TODO, account for different time zones
      //this is assumed to be unix-time in the same time-zone as the velodynes used
      time_t ts = ros::Time::now().sec;
      struct tm * timeinfo;
      timeinfo = localtime(&ts);
      double current_seconds_since_last_hour = timeinfo->tm_min*60.0 + timeinfo->tm_sec;
      ROS_INFO_STREAM("CONVERTER: current_seconds_since_last_hour = " << current_seconds_since_last_hour);

      //we have one struct with current hour and one with previous hour
      struct tm timeinfo_curr_hour = *timeinfo;
      timeinfo_curr_hour.tm_min = 0;
      timeinfo_curr_hour.tm_sec = 0;
      time_t ts_prev_h = ts-60*60;
      struct tm * timeinfo_prev_hour;
      timeinfo_prev_hour = localtime(&ts_prev_h);
      timeinfo_prev_hour->tm_min = 0;
      timeinfo_prev_hour->tm_sec = 0;

      time_t curr_h = mktime(&timeinfo_curr_hour);
      long curr_h_us = 1000000*curr_h;
      time_t prev_h = mktime(timeinfo_prev_hour);
      long prev_h_us = 1000000*prev_h;

      // process each packet provided by the driver
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
          long microseconds_since_last_hour;
          data_->unpack(scanMsg->packets[i], *outMsg, microseconds_since_last_hour);
          double seconds_since_last_hour = (double) microseconds_since_last_hour / 1.0e6;
          if(i==0)
              ROS_INFO_STREAM("CONVERTER: seconds_since_last_hour = " << seconds_since_last_hour);
          //if seconds_since_last_hour is too large, we are getting data from the previous
          //hour, so decrement the hour by one to set the timestamp
          if( floor(seconds_since_last_hour) > current_seconds_since_last_hour &&
                  current_seconds_since_last_hour < 1.0) {

              //use previous hours timestamp (to set the unixtime in microseconds timestamp)
              outMsg->header.stamp = prev_h_us + microseconds_since_last_hour;
          } else {
              outMsg->header.stamp = curr_h_us + microseconds_since_last_hour;
          }
      }

      // publish the accumulated cloud message
      ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                       << " Velodyne points, time: " << outMsg->header.stamp);

      double curr_time = (double)outMsg->header.stamp/1.0e6;
      ROS_INFO_STREAM("Publishing " << outMsg->height * outMsg->width
                      << " Velodyne points, time: " << ((int) floor(curr_time)) << "."
                      << (int)1.0e6*(curr_time-floor(curr_time)));
      output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
