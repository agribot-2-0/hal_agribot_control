#include <ros/ros.h>
#include <ros/console.h>
#include <TTComm.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    TTComm communicationHandle;

    while(ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}