#include <ros/ros.h>
#include <TTComm.h>

int main(int argc, char **argv)
{
    double start;
    ros::init(argc, argv, "ttcontrol_communication_protocol");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    TTComm commHandler;
    
    //Handshake loop
    commHandler.vSendHanshake();
    start = ros::Time::now().toSec();
    while(ros::ok())
    {
        if( (ros::Time::now().toSec()-start) >= 5)
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
    while(ros::ok())
    {
        if((commHandler.iCorrectHBReceived()==0))
        {
            bResponseToHb=true;
        }

        if(bResponseToHb && (start - ros::Time::now().toSec()) >= 0.150)
        {
            commHandler.vSendHB();
            start = ros::Time::now().toSec();
            bResponseToHb=false;
        }
        
        ros::spinOnce();
	    loop_rate.sleep();
    }

}