/*
 * Copyright (c) 2020, stauto-seoultech
 * author : yungsik
*/

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_converter/ObstacleArrayMsg.h>
//#include <vision_msgs/BoundingBox2D.h>

#define MODENUM 6;
using namespace std;

class CoreControl
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber obstacle_sub, stop_sub;
        ros::Publisher mode_pub;
        ros::Timer timer1;

        enum mode
        {  crusie,
           avoidance,
           stop,
           traffic,
           parking,
           safetyzone
        };

        std_msgs::ByteMultiArray mode_array;
        std_msgs::Bool stop_line;
        costmap_converter::ObstacleArrayMsg obst_array;
        int controller_freq;

        void ObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& ObstacleMsg);
        void StopCB(const std_msgs::Bool::ConstPtr& LineMsg);
        void controlLoopCB(const ros::TimerEvent&);

    public:
        CoreControl();
        void ModeDecision(const costmap_converter::ObstacleArrayMsg obsContainer, const std_msgs::ByteMultiArray mode);

};

CoreControl:: CoreControl()
{
    ros::NodeHandle pn("~");
    pn.param("controller_freq",controller_freq,10);

    obstacle_sub = nh.subscribe("obstacle", 1,  &CoreControl::ObstacleCB,this);
    stop_sub = nh.subscribe("stop_line", 1,  &CoreControl::StopCB,this);
    mode_pub = nh.advertise<std_msgs::ByteMultiArray>("/mode",1);

    timer1 = nh.createTimer(ros::Duration((1.0)/controller_freq), &CoreControl::controlLoopCB,this);

    for(int i = 0; i < 6 ; i++)
    {
        if(i == 0)
        {
            mode_array.data[i] = true;
        }
        else
        {
            mode_array.data[i] = false;
        }
    }
}

void CoreControl::ModeDecision(const costmap_converter::ObstacleArrayMsg obsContainer, const std_msgs::ByteMultiArray mode)
{

}

void CoreControl::ObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& ObstacleMsg)
{
    this-> obst_array = *ObstacleMsg;
}

void CoreControl::StopCB(const std_msgs::Bool::ConstPtr& LineMsg)
{
    this->stop_line = *LineMsg;
}

void CoreControl::controlLoopCB(const ros::TimerEvent&)
{
    //CoreControl::ModeDecision();

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"core_control");
    CoreControl core_control_node;
    ros::spin();
    return(0);
}