#include <ros/ros.h>
#include <ros/console.h>
#include <TTComm.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTComm commHandler;

    //Frame from application comes. 
    //Run diagnostic routine, 
    //wait for ttcontrol control request,
    //confirm it
    int iRet=-5;
    while(ros::ok())
    {
        commHandler.iReceiveCanFrames();
        commHandler.iParseCanFrames();

        
        ros::spinOnce();
        loop_rate.sleep();
    }
}