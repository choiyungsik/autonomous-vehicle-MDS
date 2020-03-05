/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <time.h>
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
#define FILTERSIZE 4

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;
double east;
double north;
double pre_x[FILTERSIZE], pre_y[FILTERSIZE];
double pre_yaw;
ros::Time ts, pre_ts, td, pre_td;
int cnt;
tf::Quaternion q;
double yaw_val_;
double f_cut = 10;
double w_cut = 2 * M_PI * f_cut;
double tau = 1 / w_cut;
double data[FILTERSIZE] = {0};


///////////////////////////////////////


double Q_angle = 0.001f;
double Q_bias = 0.003f;
double R_measure = 0.03f;

double angle = 0.0f; // Reset the angle
double bias = 0.0f; // Reset bias

double P[2][2] = {0,0,0,0};


/////////////////////////////////////////

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

    for(int i = 0; i < FILTERSIZE; i++)
    {
      data[i] = atan2(northing - north - pre_y[0], easting - east - pre_x[0]);
    }
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


    odom.header.frame_id = "map";
    odom.child_frame_id = "odom";

    // odom.pose.pose.position.x = -(northing - north) * cos(-0.30535448) + (easting - east) * sin(-0.30535448);
    // odom.pose.pose.position.y = (northing - north) * sin(-0.30535448) + (easting - east) * cos(-0.30535448);
    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = 0;

    ///////Low Pass Filter/////////

    double tmp = atan2(odom.pose.pose.position.y - pre_y[0], odom.pose.pose.position.x - pre_x[0] );

    ts = ros::Time::now();
    double time = ts.toSec() - pre_ts.toSec();
    double Yaw =(tau * (pre_yaw + M_PI) + time * (tmp + M_PI)) / (tau + time) - M_PI;
    pre_ts = ts;

    ///////////////////////////////
    ////////////////////////////////

    /* Step 1 */
    double newRate = (Yaw - pre_yaw) /time;
    double rate = newRate - bias;
    angle += time * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += time * (time * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= time * P[1][1];
    P[1][0] -= time * P[1][1];
    P[1][1] += Q_bias * time;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = Yaw - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    ////////////////////////////////

    //////MoveAverageFilter////////

    // double sum = 0, average = 0;

    // for(int i = 0; i < FILTERSIZE; i++)
    // {
    //   sum += data[i];

    // }

    // double semi_average = sum/FILTERSIZE;

    // average = sum/(FILTERSIZE + 1);

    // for(int i = 0; i < FILTERSIZE - 1; i++)
    // {
    //   data[i] = data[i+1];
    // }

    // data[FILTERSIZE - 1] = average;
    // double Yaw2 = average;

    ///////////////////////////////
    pre_yaw = angle;
    ROS_INFO("%.4f",pre_yaw);
    //double Yaw = atan2(odom.pose.pose.position.y - pre_y, odom.pose.pose.position.x - pre_x );


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0) );
    q.setRPY(0, 0, angle);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    odom.pose.pose.orientation.w = q.getW();



    for(int i = 0; i < FILTERSIZE - 1; i++)
    {
      pre_x[i] = pre_x[i+1];
      pre_y[i] = pre_y[i+1];
    }

    pre_x[FILTERSIZE - 1] = easting - east;
    pre_y[FILTERSIZE - 1] = northing - north;




    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw, yaw_;

    // m.getRPY(roll, pitch, yaw);
    // yaw_ = -yaw_val_*0.9 - 85 * (3.14/180); // because our imu is not right hand rule and align

    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw_);
    // geometry_msgs::Quaternion odom_quat_;
    // if(isnan(odom_quat.x) != true){
    //   odom.pose.pose.orientation = odom_quat;
    //   odom_quat_ = odom_quat;
    // }
    // else{
    //   odom.pose.pose.orientation = odom_quat_;
    // }

    // q.setRPY(roll, pitch, yaw_);
    // transform.setRotation(q);
    // transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
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

  odom_pub = node.advertise<nav_msgs::Odometry>("odom/gps", 1);

  ros::Subscriber fix_sub = node.subscribe("gps/fix", 1, callback);
   ros::Subscriber imu_sub = node.subscribe("/imu/data", 10, imu_callback);
   ros::Subscriber yaw_sub = node.subscribe("yaw_degree", 10, yaw_callback);

  ros::spin();
}
