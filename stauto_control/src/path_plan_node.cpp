/*
 * Copyright (c) 2020, stauto-seoultech.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <geometry_msgs/PoseStamped.h>
//#include <vision_msgs/BoundingBox2D.h>

using namespace std;

class PathPlan
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber global_path_sub,lane_path_sub,gps_accuracy_sub,lane_accuracy_sub,robot_step_sub; //object_info
        ros::Publisher local_path_pub;

        nav_msgs::Path global_path,lane_path,local_path;
        std_msgs::UInt32 gps_accuracy, lane_accuracy;
        geometry_msgs::PoseStamped robot_step;


        void GlobalCB(const nav_msgs::Path::ConstPtr& GpathMsg);
        void LaneCB(const nav_msgs::Path::ConstPtr& LpathMsg);
        void GaccuracyCB(const std_msgs::UInt32::ConstPtr& GaccuracyMsg );
        void LaccuracyCB(const std_msgs::UInt32::ConstPtr& LaccuracyMsg );
        void RobotStepCB(const geometry_msgs::PoseStamped::ConstPtr& RobotStepMsg);
        //void controlLoopCB(const ros::TimerEvent&);
        //oid ObjectCB(const nav_msgs::Path::ConstPtr& GpathMsg);

    public:
        PathPlan();
        nav_msgs::Path LocalPathPlan();

};

PathPlan::PathPlan()
{
    global_path_sub = nh.subscribe("global_path", 1, &PathPlan::GlobalCB,this);
    lane_path_sub = nh.subscribe("lane_path", 1, &PathPlan::LaneCB,this);
    gps_accuracy_sub = nh.subscribe("gps_accuracy", 1, &PathPlan::GaccuracyCB,this);
    lane_accuracy_sub = nh.subscribe("lane_accuracy", 1, &PathPlan::LaccuracyCB,this);
    robot_step_sub = nh.subscribe("robot_step", 1, &PathPlan::RobotStepCB,this);

    local_path = nav_msgs::Path();
}

void PathPlan::GlobalCB(const nav_msgs::Path::ConstPtr& GpathMsg)
{
    this->global_path = *GpathMsg;
}

void PathPlan::LaneCB(const nav_msgs::Path::ConstPtr& LpathMsg)
{
    this->lane_path = *LpathMsg;
}

void PathPlan::GaccuracyCB(const std_msgs::UInt32::ConstPtr& GaccuracyMsg )
{
    this->gps_accuracy = *GaccuracyMsg;
}

void PathPlan::LaccuracyCB(const std_msgs::UInt32::ConstPtr& LaccuracyMsg )
{
    this->lane_accuracy = *LaccuracyMsg;
}

void PathPlan::RobotStepCB(const geometry_msgs::PoseStamped::ConstPtr& RobotStepMsg)
{
    this -> robot_step = *RobotStepMsg;

}


//void PathPlan::controlLoopCB(const ros::TimerEvent&) {}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"path_plan_node");
    PathPlan path_plan_node;
    ros::spin();
    return(0);
}