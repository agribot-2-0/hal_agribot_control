#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <TTComm.h>

TTComm commHandler;

void frontCallback(const agribot_msgs::AgribotCaterpillar msg)
{
    ROS_INFO("Front left: [%f]", msg.front_left);
    ROS_INFO("Front right: [%f]", msg.front_right);
    ROS_INFO("Back left: [%f]", msg.back_left);
    ROS_INFO("Back right: [%f]", msg.back_right);
    commHandler.vSendVelocities(msg.front_left, msg.front_right, msg.back_left, msg.back_right);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
    uint8_t u8Status=0x00;

    //Frame from application comes. 
    //Run diagnostic routine, 
    //wait for ttcontrol control request,
    //confirm it
    /*
    while(!commHandler.bWaritForTTControlReadiness())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    commHandler.vConfirmAdvantechReadiness();
    */

    int iRet=-5;
    while(ros::ok())
    {
        commHandler.vReceiveCanFrames();
        u8Status=commHandler.u8ParseCanFrames();
        
        if((u8Status&VELOCITIES_MASK)==VELOCITIES_MASK)
        {
           std::cout << "Velocities received" << std::endl;
        }
        if((u8Status&FRONT_ANGLE_MASK)==FRONT_ANGLE_MASK)
        {
           std::cout << "Front received" << std::endl;
        }
        if((u8Status&BACK_ANGLE_MASK)==BACK_ANGLE_MASK)
        {
           std::cout << "Angles received" << std::endl;
        }
        if((u8Status&PROTOCOL_MASK)==PROTOCOL_MASK)
        {
           std::cout << "--- Protocol ---" << std::endl;
        }

        std::cout << "Next loop" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
}