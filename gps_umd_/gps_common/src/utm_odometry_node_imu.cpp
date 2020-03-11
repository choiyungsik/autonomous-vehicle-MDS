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
#include <std_msgs/Float32.h>
#include <iostream>
#include <math.h>
#include <float.h> 

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;
double east;
double north;
int cnt;
tf::Quaternion q;
double yaw_val_;

void imu_callback(const sensor_msgs::ImuConstPtr& imu){

  q = tf::Quaternion (imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w);
}

void yaw_callback(const std_msgs::Float32ConstPtr &yaw_val){

  yaw_val_ = yaw_val->data;
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

    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = fix->altitude;
    
    // odom.pose.pose.orientation.x = 0;
    // odom.pose.pose.orientation.y = 0;
    // odom.pose.pose.orientation.z = 0;
    // odom.pose.pose.orientation.w = 1;
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_;
    
    m.getRPY(roll, pitch, yaw);
    yaw_ = -yaw_val_*0.9 - 85 * (3.14/180); // because our imu is not right hand rule and align

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw_);
    geometry_msgs::Quaternion odom_quat_;
    if(isnan(odom_quat.x) != true){
      odom.pose.pose.orientation = odom_quat;
      odom_quat_ = odom_quat;
    }
    else{
      odom.pose.pose.orientation = odom_quat_;
    }

    q.setRPY(roll, pitch, yaw_);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));  // gps odom only with imu_yaw

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

  odom_pub = node.advertise<nav_msgs::Odometry>("/odom/gps", 10);

  ros::Subscriber fix_sub = node.subscribe("/gps/fix", 10, callback);
  ros::Subscriber imu_sub = node.subscribe("/imu/data", 10, imu_callback);
  ros::Subscriber yaw_sub = node.subscribe("yaw_degree", 10, yaw_callback);

  ros::spin();
}
