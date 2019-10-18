#include <ros/ros.h>
#include <ros/console.h>
#include <TTCommHB.h>

int main(int argc, char **argv)
{
    double dStart;
    ros::init(argc, argv, "heartbeat");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTCommHB CommHandlerHB;

    CommHandlerHB.vSendHanshake();
    dStart = ros::Time::now().toSec();
    while(ros::ok())
    {
        if((ros::Time::now().toSec()-dStart) >= 5)
        {
            ROS_WARN("[HB Node]No response from TTControl. Resending handshake");
            CommHandlerHB.vSendHanshake();
            dStart = ros::Time::now().toSec();
        }
        if(CommHandlerHB.bWaitForHandshake())
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[HB Node]Handshake completed. Entering HB phase.");
    CommHandlerHB.vSendHB();
    dStart = ros::Time::now().toSec();
    bool bResponseToHb =false;
    int iRet = -5;
    while(ros::ok())
    {
        iRet = CommHandlerHB.iHBReceived();
        if(iRet==0)
        {
            bResponseToHb=true;
        }
        else if(iRet == -2)
        {
            ROS_ERROR("[HB Node] Wrong heart beat value received");
            return -1;
        }
        if(bResponseToHb && (ros::Time::now().toSec() - dStart) >= 0.150)
        {
            CommHandlerHB.vSendHB();
            dStart = ros::Time::now().toSec();
            bResponseToHb=false;
        }

        if((ros::Time::now().toSec() - dStart) > 0.320)
        {
            ROS_ERROR("[HB Node] Herat beat timeout");
            return -2;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}