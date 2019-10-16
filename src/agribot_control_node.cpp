#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <TTComm.h>

TTComm commHandler;

void frontCallback(const agribot_msgs::AgribotCaterpillar msg)
{
    ROS_INFO("I heard: [%f]", msg.left);
    ROS_INFO("I heard: [%f]", msg.right);
    commHandler.vSendVelocities(msg.left, msg.right);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar/front", 1000, frontCallback);
   
    //Frame from application comes. 
    //Run diagnostic routine, 
    //wait for ttcontrol control request,
    //confirm it
    int iRet=-5;
    while(ros::ok())
    {
        commHandler.vReceiveCanFrames();
        commHandler.iParseCanFrames();

        ros::spinOnce();
        loop_rate.sleep();
    }
}