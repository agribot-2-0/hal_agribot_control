#include <ros/ros.h>
#include <ros/console.h>
#include <hal_ttcontrol_communication/TTCommHB.h>

#define RESEND_HANDSHAKE_INTERVAL 5
#define HB_RESPONSE_INTERVAL 0.150
#define HB_TIMEOUT 0.320

int main(int argc, char **argv)
{
    double dStart;
    ros::init(argc, argv, "heartbeat");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTCommHB CommHandlerHB("can0");

    CommHandlerHB.vSendHanshake();
    dStart = ros::Time::now().toSec();
    while(ros::ok())
    {
        //if there is no response after RESEND_HANDSHAKE_INTERVAL seconds
        if((ros::Time::now().toSec()-dStart) >= RESEND_HANDSHAKE_INTERVAL)
        {
            ROS_WARN("[HB Node] No response from TTControl. Resending handshake");
            CommHandlerHB.vSendHanshake();
            dStart = ros::Time::now().toSec();
        }
        //if TTC "shook our hand"
        if(CommHandlerHB.bIsHandshakeReceived())
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[HB Node] Handshake completed. Entering HB phase.");
    CommHandlerHB.vSendHB();
    dStart = ros::Time::now().toSec();
    bool bRespondToHb = false;
    int iRet = -5;
    while(ros::ok())
    {
        iRet = CommHandlerHB.iHBReceived();
        //if HB received in iHBReceived() method is correct
        if(iRet == CORRECT_HB_RECEIVED)
        {
            bRespondToHb = true;
        }
        //if HB received in iHBReceived() in wrong
        else if(iRet == WRONG_HB_RECEIVED)
        {
            ROS_ERROR("[HB Node] Wrong heart beat value received");
            return -1;
        }
        //if HB is received and HB_RESPONSE_INTERVAL seconds passed
        if(bRespondToHb && (ros::Time::now().toSec() - dStart) >= HB_RESPONSE_INTERVAL)
        {
            CommHandlerHB.vSendHB();
            dStart = ros::Time::now().toSec();
            bRespondToHb = false;
        }

        //if TTC dint respond in HB_TIMEOUT seconds
        if((ros::Time::now().toSec() - dStart) > HB_TIMEOUT)
        {
            ROS_ERROR("[HB Node] Herat beat timeout");
            return -2;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}