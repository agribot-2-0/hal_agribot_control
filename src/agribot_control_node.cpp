#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <std_msgs/Float32.h>
#include <TTComm.h>

TTComm commHandler;

void frontCallback(const agribot_msgs::AgribotCaterpillar msg)
{
    ROS_INFO("Front linear: [%f]", msg.front_linear);
    ROS_INFO("Front angular: [%f]", msg.front_angular);
    ROS_INFO("Back linear: [%f]", msg.back_linear);
    ROS_INFO("Back angular: [%f]", msg.back_angular);
    commHandler.vSendVelocities(msg.front_linear, msg.front_angular, msg.back_linear, msg.back_angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
    ros::Publisher CartsStatusPublisher = n.advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar",1);
    ros::Publisher FrontCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front",1);
    ros::Publisher BackCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back",1);
    agribot_msgs::AgribotCaterpillar AgribotCaterpillarMsg;
    std_msgs::Float32 FrontAngleMsg;
    std_msgs::Float32 BackAngleMsg;
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
            //Publish velicities
            std::cout << "Velocities received" << std::endl;
            CartsStatus cartStatus = commHandler.GetCartsStatus();
            AgribotCaterpillarMsg.front_linear = cartStatus.fGetFrontLinear();
            AgribotCaterpillarMsg.front_angular = cartStatus.fGetFrontAngular();
            AgribotCaterpillarMsg.back_linear = cartStatus.fGetBackLinear();
            AgribotCaterpillarMsg.back_angular = cartStatus.fGetBackAngular();
            CartsStatusPublisher.publish(AgribotCaterpillarMsg);            
        }
        if((u8Status&FRONT_ANGLE_MASK)==FRONT_ANGLE_MASK)
        {
            //Publish front angle
            std::cout << "Front angle received" << std::endl;
            FrontAngleMsg.data = commHandler.fGetFrontCartAngle();
            FrontCartAnglePublisher.publish(FrontAngleMsg);
        }
        if((u8Status&BACK_ANGLE_MASK)==BACK_ANGLE_MASK)
        {
            //Publish front angle
            std::cout << "Back angle received" << std::endl;
            BackAngleMsg.data = commHandler.fGetBackCartAngle();
            BackCartAnglePublisher.publish(BackAngleMsg);
        }
        if((u8Status&PROTOCOL_MASK)==PROTOCOL_MASK)
        {
            //React properly to protocol frame
            std::cout << "--- Protocol ---" << std::endl;
        }

        std::cout << "Next loop" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
}
