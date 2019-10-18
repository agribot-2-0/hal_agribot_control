#include <ros/ros.h>
#include <ros/console.h>
#include <TTCommHB.h>

int main(int argc, char **argv)
{
    double start;
    ros::init(argc, argv, "ttcontrol_communication_protocol");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTCommHB commHandler;
    
    //Handshake loop
    commHandler.vSendHanshake();
    start = ros::Time::now().toSec();
    while(ros::ok())
    {
        if((ros::Time::now().toSec()-start) >= 5)
        {
            std::cout << "Resending handshake" << std::endl;
            commHandler.vSendHanshake();
            start = ros::Time::now().toSec();
        }
        
        if(commHandler.bWaitForHandshake())
        {
            std::cout <<"response recived!" << std::endl;
            break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "enter HB phase:" << std::endl;
    commHandler.vSendHB();
    start = ros::Time::now().toSec();
    bool bResponseToHb =false;
    int iRet = -5;
    while(ros::ok())
    {
        iRet = commHandler.iCorrectHBReceived();
        if(iRet==0)
        {
            bResponseToHb=true;
        }
        else if(iRet == -2)
        {
            ROS_ERROR("[TTControl] Wrong heart beat value received");
            return -1;
        }

        if(bResponseToHb && (ros::Time::now().toSec() - start) >= 0.150)
        {
            commHandler.vSendHB();
            start = ros::Time::now().toSec();
            bResponseToHb=false;
        }

        if((ros::Time::now().toSec() - start) > 0.320)
        {
            ROS_ERROR("[TTControl] Herat beat timeout");
            return -2;
        }
        
        ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}