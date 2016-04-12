#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <libgpsmm.h>


#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


/**
 * NavSatFix messages do not include measured speed and course (angle relative to true north) which IS
 * supported by many GPS units (e.g. trimble)
 */

using namespace gps_common;
using namespace sensor_msgs;

static ros::Publisher odom_pub;
static tf::TransformBroadcaster * odom_broadcaster;
std::string frame_id, child_frame_id;

class GPSDClient {
  public:
    GPSDClient() : privnode("~"), gps(NULL), use_gps_time(true), check_fix_by_variance(true),last_OK_course(0.0) {}

    bool start() {
      gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
      navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);

      privnode.getParam("use_gps_time", use_gps_time);
      privnode.getParam("check_fix_by_variance", check_fix_by_variance);

      std::string host = "localhost";
      int port = 2947;
      privnode.getParam("host", host);
      privnode.getParam("port", port);

      char port_s[12];
      snprintf(port_s, 12, "%d", port);

      gps_data_t *resp = NULL;

#if GPSD_API_MAJOR_VERSION >= 5
      gps = new gpsmm(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
      gps = new gpsmm();
      gps->open(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#else
      gps = new gpsmm();
      resp = gps->open(host.c_str(), port_s);
      gps->query("w\n");
#endif

      if (resp == NULL) {
        ROS_ERROR("Failed to open GPSd");
        return false;
      }

      ROS_INFO("GPSd opened");
      return true;
    }

    void step() {
#if GPSD_API_MAJOR_VERSION >= 5
      if (!gps->waiting(1e6)) {
          ROS_INFO_STREAM("Gps->waiting!");
          return;
      }

      gps_data_t *p = gps->read();
      ROS_INFO_STREAM("gps->read");

      ROS_INFO_STREAM("gps->online = " << (p->online ? "true":"false") << " gps->fix->lat, lon "<<p->fix.latitude << ", " << p->fix.longitude);

      //#define STATUS_NO_FIX	0	/* no */
      //#define STATUS_FIX	1	/* yes, without DGPS */
      //#define STATUS_DGPS_FIX	2	/* yes, with DGPS */
      ROS_INFO_STREAM("gps->status = " << p->status << " (0-NO_FIX, 1-FIX W/O DGPS, 2-FIX DGPS)" );
#else
      gps_data_t *p = gps->poll();
      ROS_INFO_STREAM("gps->poll");
#endif
      process_data(p);
    }

    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    ros::Publisher gps_fix_pub;
    ros::Publisher navsat_fix_pub;
    gpsmm *gps;

    bool use_gps_time;
    bool check_fix_by_variance;

    double last_OK_course;

    void process_data(struct gps_data_t* p) {
      if (p == NULL)
        return;

      if (!p->online)
        return;

      process_data_gps(p);
      process_data_navsat(p);
    }


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif

    void process_data_gps(struct gps_data_t* p) {
      ros::Time time = ros::Time::now();

      GPSFix fix;
      GPSStatus status;

      //could change to use
      //p->fix.time; instead of rostime::now
      status.header.stamp = time;
      fix.header.stamp = time;
      fix.header.frame_id = child_frame_id;

      status.satellites_used = p->satellites_used;

      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
        status.satellite_used_prn[i] = p->used[i];
      }

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      for (int i = 0; i < SATS_VISIBLE; ++i) {
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
        status.satellite_visible_snr[i] = p->ss[i];
      }

      ROS_INFO_STREAM("p->fix.track = " << p->fix.track ); //this sucks unless we are moving!
      ROS_INFO_STREAM("p->fix.speed = " << p->fix.speed); //this also
      ROS_INFO_STREAM("p->fix.epx = " << p->fix.epx);
      if ((p->status & STATUS_FIX) && !(check_fix_by_variance && isnan(p->fix.epx))) {
        status.status = 0; // FIXME: gpsmm puts its constants in the global
                           // namespace, so `GPSStatus::STATUS_FIX' is illegal.

        if (p->status & STATUS_DGPS_FIX)
          status.status |= 18; // same here

        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      } else {
      	status.status = -1; // STATUS_NO_FIX
      }

      fix.status = status;

      ROS_INFO_STREAM("Publishing on /extended_fix !");
      if(!isnan(fix.latitude) && !isnan(fix.longitude))
          gps_fix_pub.publish(fix);

      //Do conversion to UTM here, and publish nav_msgs::Odometry and TF
      ///TODO should use the same conversion code for everything... (to avoid strange bugs)
      double northing, easting;
      std::string zone;
      LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);



      if (odom_pub) {
        nav_msgs::Odometry odom;
        odom.header.stamp = fix.header.stamp;

        if (frame_id.empty())
          odom.header.frame_id = fix.header.frame_id;
        else
          odom.header.frame_id = frame_id;

        odom.child_frame_id = child_frame_id;

        odom.pose.pose.position.x = easting;
        odom.pose.pose.position.y = northing;
        odom.pose.pose.position.z = fix.altitude;

        /* Course made good (relative to true north) */

        //angle in degrees, clockwise from north, we want angle in radians counter-clockwise from east
        double courseTrueNorthDeg = p->fix.track;
        if(!isnan(courseTrueNorthDeg)) {
            last_OK_course = courseTrueNorthDeg;
        }
        double yaw = M_PI/180.0*(90-last_OK_course);

        tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw);

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Use ENU covariance to build XYZRPY covariance
        double rot_cov = 999; ///TODO

        //this can't be correct... should map this through lon,lat -> utm conversion...
        boost::array<double, 36> covariance = {{
          fix.position_covariance[0],
          fix.position_covariance[1],
          fix.position_covariance[2],
          0, 0, 0,
          fix.position_covariance[3],
          fix.position_covariance[4],
          fix.position_covariance[5],
          0, 0, 0,
          fix.position_covariance[6],
          fix.position_covariance[7],
          fix.position_covariance[8],
          0, 0, 0,
          0, 0, 0, rot_cov, 0, 0,
          0, 0, 0, 0, rot_cov, 0,
          0, 0, 0, 0, 0, rot_cov
        }};
        odom.pose.covariance = covariance;



        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = odom.header.stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;
        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.translation.z = odom.pose.pose.position.z;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();


        if( !isnan(easting) && !isnan(northing) && !isnan(fix.altitude) ) {
            odom_pub.publish(odom);
            odom_broadcaster->sendTransform(odom_trans);
        }
      }

    }

    void process_data_navsat(struct gps_data_t* p) {
      NavSatFixPtr fix(new NavSatFix);

      /* TODO: Support SBAS and other GBAS. */

      if (use_gps_time && !isnan(p->fix.time))
        fix->header.stamp = ros::Time(p->fix.time);
      else
        fix->header.stamp = ros::Time::now();

      fix->header.frame_id = child_frame_id;

      /* gpsmm pollutes the global namespace with STATUS_,
       * so we need to use the ROS message's integer values
       * for status.status
       */
      switch (p->status) {
        case STATUS_NO_FIX:
          fix->status.status = -1; // NavSatStatus::STATUS_NO_FIX;
          break;
        case STATUS_FIX:
          fix->status.status = 0; // NavSatStatus::STATUS_FIX;
          break;
        case STATUS_DGPS_FIX:
          fix->status.status = 2; // NavSatStatus::STATUS_GBAS_FIX;
          break;
      }

      fix->status.service = NavSatStatus::SERVICE_GPS;

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */
      if (isnan(p->fix.epx) && check_fix_by_variance) {
        return;
      }

      fix->position_covariance[0] = p->fix.epx;
      fix->position_covariance[4] = p->fix.epy;
      fix->position_covariance[8] = p->fix.epv;

      fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      if(!isnan(fix->latitude) && !isnan(fix->longitude))
          navsat_fix_pub.publish(fix);
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
  tf::TransformBroadcaster tfBC;
  odom_broadcaster = &tfBC;

  GPSDClient client;

  if (!client.start())
    return -1;


  while(ros::ok()) {
    ros::spinOnce();
    client.step();
  }

  client.stop();
}
