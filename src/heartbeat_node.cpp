#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <hal_ttcontrol_communication/TTCommHB.h>
#include <string>

#define RESEND_HANDSHAKE_INTERVAL 5
#define HB_RESPONSE_INTERVAL 0.150
#define HB_TIMEOUT 0.320
#define USED_CAN "can0"

int main(int argc, char **argv)
{
    double dStart;
    ros::init(argc, argv, "heartbeat");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTCommHB CommHandlerHB(USED_CAN);

    ros::Publisher ConnectionStatePublisher = n.advertise<std_msgs::String>("/agribot/ttcontrol/connection_state",1);
    std_msgs::String msg;
    msg.data="connecting";
    while(ros::ok())
    {
        msg.data="connecting";
        ConnectionStatePublisher.publish(msg);
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
                msg.data="connecting";
                ConnectionStatePublisher.publish(msg);
            }
            //if TTC "shook our hand"
            if(CommHandlerHB.bIsHandshakeReceived())
            {
                msg.data="connected";
                ConnectionStatePublisher.publish(msg);
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
                msg.data="connected";
                ConnectionStatePublisher.publish(msg);
                bRespondToHb = true;
            }
            //if HB received in iHBReceived() in wrong
            else if(iRet == WRONG_HB_RECEIVED)
            {
                ROS_ERROR("[HB Node] Wrong heart beat value received");
                msg.data = "disconnected";
                ConnectionStatePublisher.publish(msg);
                break;
            }
            //if HB is received and HB_RESPONSE_INTERVAL seconds passed
            if(bRespondToHb && (ros::Time::now().toSec() - dStart) >= HB_RESPONSE_INTERVAL)
            {
                CommHandlerHB.vSendHB();
                dStart = ros::Time::now().toSec();
                msg.data = "connected";
                ConnectionStatePublisher.publish(msg);
                bRespondToHb = false;
            }

            //if TTC dint respond in HB_TIMEOUT seconds
            if((ros::Time::now().toSec() - dStart) > HB_TIMEOUT)
            {
                ROS_ERROR("[HB Node] Heart beat timeout");
                msg.data = "disconnected";
                ConnectionStatePublisher.publish(msg);
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}