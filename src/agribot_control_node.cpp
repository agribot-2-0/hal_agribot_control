#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <std_msgs/Float32.h>
#include <hal_ttcontrol_communication/TTCommControl.h>

TTCommControl CommHandlerControl;

void frontCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
    CommHandlerControl.vSendVelocities(msg.front_linear, msg.front_angular, msg.back_linear, msg.back_angular);
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

    while(ros::ok())
    {
        CommHandlerControl.vReceiveCanFrames();
        u8Status=CommHandlerControl.u8ParseCanFrames();
        if((u8Status&VELOCITIES_MASK) == VELOCITIES_MASK)
        {
            CartsStatus cartStatus = CommHandlerControl.GetCartsStatus();
            AgribotCaterpillarMsg.front_linear = cartStatus.fGetFrontLinear();
            AgribotCaterpillarMsg.front_angular = cartStatus.fGetFrontAngular();
            AgribotCaterpillarMsg.back_linear = cartStatus.fGetBackLinear();
            AgribotCaterpillarMsg.back_angular = cartStatus.fGetBackAngular();
            CartsStatusPublisher.publish(AgribotCaterpillarMsg);
        }
        if((u8Status&FRONT_ANGLE_MASK) == FRONT_ANGLE_MASK)
        {
            FrontAngleMsg.data = CommHandlerControl.fGetFrontCartAngle();
            FrontCartAnglePublisher.publish(FrontAngleMsg);
        }
        if((u8Status&BACK_ANGLE_MASK) == BACK_ANGLE_MASK)
        {
            BackAngleMsg.data = CommHandlerControl.fGetBackCartAngle();
            BackCartAnglePublisher.publish(BackAngleMsg);
        }
        if((u8Status&AUTOMATIC_STATE_REQUEST_MASK) == AUTOMATIC_STATE_REQUEST_MASK)
        {
            //TODO: Implement some diagnostics
            CommHandlerControl.vConfirmAdvantechReadiness();
        }
        if((u8Status&AUTOMATIC_STATE_REQUEST_ERROR_MASK) == AUTOMATIC_STATE_REQUEST_ERROR_MASK)
        {
            ROS_ERROR("Automatic state request error: %d", CommHandlerControl.iGetErrorCode());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
