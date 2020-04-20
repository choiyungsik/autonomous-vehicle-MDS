#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"final_control");
    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::Rate loop_rate(10);


}