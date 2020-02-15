/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>


using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;
double east;
double north;
int cnt;
double yaw;
tf::Quaternion q;

void imu_callback(const sensor_msgs::ImuConstPtr& imu){
  // tf2::convert(imu->orientation , q);
}

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_DEBUG_THROTTLE(60,"No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;
  

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if(cnt == 0)
  {
    east = easting;
    north = northing;
    cnt++;
  }
  else
  {
    
  }

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty()) {
      if(append_zone) {
        odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = fix->header.frame_id;
      }
    } else {
      if(append_zone) {
        odom.header.frame_id = frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = frame_id;
      }
    }


    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = -(northing - north) * cos(-0.30535448) + (easting - east) * sin(-0.30535448);
    odom.pose.pose.position.y = (northing - north) * sin(-0.30535448) + (easting - east) * cos(-0.30535448);
    odom.pose.pose.position.z = fix->altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    odom.twist.twist.angular.z = yaw;

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transform.setRotation(q);
    // transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "odom"));  

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("append_zone", append_zone, false);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  ros::Subscriber fix_sub = node.subscribe("/gps/fix", 10, callback);
  ros::Subscriber imu_sub = node.subscribe("/imu/data", 10, imu_callback);

  ros::spin();
}

