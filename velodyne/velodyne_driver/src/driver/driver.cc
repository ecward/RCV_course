/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "driver.h"
#include <time.h>

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
    private_nh.param("stream_id",stream_id_,std::string(""));
    bool offline_mode = private_nh.param("pcap", std::string("")) != "";
    ROS_INFO_STREAM("Starting velodyne driver id = " << stream_id_ << " running in " << (offline_mode ? "offline mode":"online mode"));

  // use private node handle to get parameters
  //private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  config_.frame_id = "velodyne_"+stream_id_;
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);




  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") || 
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 781.25;             // 300000 / 384
      model_full_name = "VLP-16";
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int)UDP_PORT_NUMBER);

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets_"+stream_id_, diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")
  {
      input_.reset(new velodyne_driver::InputPCAP(private_nh,
                                                  packet_rate,
                                                  dump_file));
  }
  else
  {
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
  }

  ///TODO, clean this up to be more transparent with regards to PCAP or not..
  //Only care about this if we are using pcap
  if( dump_file != "") {
      std::string dest_port;
      private_nh.param("port",dest_port,std::string(""));
      if(!dest_port.empty())
          ROS_INFO_STREAM("Set port filter to default " << dest_port);
      else {
          ROS_INFO_STREAM("Set port filter to " << udp_port);
          std::stringstream ss;
          ss << udp_port;
          dest_port = ss.str();
      }
      input_->setDesinationPort(dest_port);
  }
  std::string devip;
  private_nh.param("device_ip", devip, std::string(""));                    
  if(!devip.empty())
      ROS_INFO_STREAM("Set device ip to " << devip << ", only accepting packets from this address." );
  else
      ROS_WARN_STREAM("Device ip not set!");
  input_->setDeviceIP(devip);


  // raw data output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets_"+stream_id_, 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i]);
          if (rc == 0) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
        }
    }

  // publish message using time of last packet read
  //ROS_DEBUG("Publishing a full Velodyne scan.");
  ROS_INFO_STREAM("Publish velodyne scan at ros::Time = "<<ros::Time::now());

  ///TODO... need to break out raw-data into it's own lib
  /// We shouldn't neeed this... The pcap poller should
  /// just publish stuff according to a specific rate...
  //We need to have the correct time here also, shieeet??

  //&scan->packets[config_.npackets-1];
  const raw_packet_vlp16_t * raw = (const raw_packet_vlp16_t *) &(scan->packets[0].data[0]);
  uint32_t * ts_data = (uint32_t*) &(raw->timestamp[0]);
  long microseconds_since_last_hour = (long)*ts_data;

  //this is assumed to be unix-time in the same time-zone as the velodynes used
  time_t ts = ros::Time::now().sec;
  struct tm * timeinfo;
  timeinfo = localtime(&ts);
  double current_seconds_since_last_hour = timeinfo->tm_min*60.0 + timeinfo->tm_sec;
  ROS_INFO_STREAM("DRIVER: current_seconds_since_last_hour = " << current_seconds_since_last_hour);

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
  double seconds_since_last_hour = (double) microseconds_since_last_hour / 1.0e6;
  ROS_INFO_STREAM("DRIVER: seconds_since_last_hour = " << seconds_since_last_hour);
  long msg_stamp=0;
  if( floor(seconds_since_last_hour) > current_seconds_since_last_hour &&
          current_seconds_since_last_hour < 1.0) {

      //use previous hours timestamp (to set the unixtime in microseconds timestamp)
      msg_stamp = prev_h_us + microseconds_since_last_hour;
  } else {
      msg_stamp = curr_h_us + microseconds_since_last_hour;
  }


  scan->header.stamp = ros::Time( (double)msg_stamp/1.0e6 );
  ROS_INFO_STREAM("Next scan timestamp = "<< scan->header.stamp.sec << "." << scan->header.stamp.nsec);
  ///END TODO

  scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp);
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);  
  diagnostics_.update();

  return true;
}

} // namespace velodyne_driver
