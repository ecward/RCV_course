/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <velodyne_driver/input.h>
#include <inttypes.h>
#include <velodyne_msgs/ExternalTimeSource.h>
#include <velodyne_msgs/VelodyneScan.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);

private:

  bool getPackets(velodyne_msgs::VelodyneScanPtr scan);
  void printTimeInfo(const std::string &description, long timestamp);
  void updateSimtime(const velodyne_msgs::ExternalTimeSource::ConstPtr &timeSourceMsg);

  double sim_time_;
  double sys_time_for_sim_time_;
  ros::Subscriber time_source_;

  std::string stream_id_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
  } config_;

  bool offline_mode_;
  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

} // namespace velodyne_driver

///TODO REMOVE
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
typedef struct raw_block
{
  uint16_t header;        ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
  uint8_t  data[BLOCK_DATA_SIZE];
} raw_block_t;
/**
 * Raw Velodyne packet constants and structures.
 */
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int TIMESTAMP_SIZE = 4;
static const int FACTORY_SIZE = 2;

typedef struct raw_packet_vlp16
{
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint8_t timestamp[TIMESTAMP_SIZE];
    uint8_t factory[FACTORY_SIZE];

} raw_packet_vlp16_t;

#endif // _VELODYNE_DRIVER_H_
