/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <std_msgs/Float32.h>
#include <hal_ttcontrol_communication/TTCommControl.h>

#define USED_CAN "can0"
#define WAIT_FOR_TTCONTROL 100000

TTCommControl CommHandlerControl(USED_CAN);

void frontCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
    CommHandlerControl.vSendVelocities(msg.front_linear, msg.front_angular,
                                       msg.back_linear, msg.back_angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub =
        n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
    ros::Publisher CartsStatusPublisher =
        n.advertise<agribot_msgs::AgribotCaterpillar>
        ("/agribot/encoder/caterpillar", 1);
    ros::Publisher FrontCartAnglePublisher =
        n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
    ros::Publisher BackCartAnglePublisher =
        n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
    agribot_msgs::AgribotCaterpillar AgribotCaterpillarMsg;
    std_msgs::Float32 FrontAngleMsg;
    std_msgs::Float32 BackAngleMsg;
    uint8_t u8Status = 0x00;

    while (ros::ok())
    {
        CommHandlerControl.vReceiveCanFrames();
        u8Status = CommHandlerControl.u8ParseCanFrames();
        // if velociteis frames received in u8ParseCanFrames() method
        if ((u8Status&VELOCITIES_MASK) == VELOCITIES_MASK)
        {
            CartsStatus cartStatus = CommHandlerControl.GetCartsStatus();
            AgribotCaterpillarMsg.header.stamp = ros::Time::now();
            AgribotCaterpillarMsg.header.frame_id = "hal_caterpillar_feedback";
            AgribotCaterpillarMsg.front_linear = cartStatus.fGetFrontLinear();
            AgribotCaterpillarMsg.front_angular = cartStatus.fGetFrontAngular();
            AgribotCaterpillarMsg.back_linear = cartStatus.fGetBackLinear();
            AgribotCaterpillarMsg.back_angular = cartStatus.fGetBackAngular();
            CartsStatusPublisher.publish(AgribotCaterpillarMsg);
        }
        // if front cart angle frames received in u8ParseCanFrames() method
        if ((u8Status&FRONT_ANGLE_MASK) == FRONT_ANGLE_MASK)
        {
            FrontAngleMsg.data = CommHandlerControl.fGetFrontCartAngle();
            FrontCartAnglePublisher.publish(FrontAngleMsg);
        }
        // if back cart angle frames received in u8ParseCanFrames() method
        if ((u8Status&BACK_ANGLE_MASK) == BACK_ANGLE_MASK)
        {
            BackAngleMsg.data = CommHandlerControl.fGetBackCartAngle();
            BackCartAnglePublisher.publish(BackAngleMsg);
        }

        // if automatic state request frame received in u8ParseCanFrames()
        // method
        if ((u8Status&AUTOMATIC_STATE_REQUEST_MASK) ==
             AUTOMATIC_STATE_REQUEST_MASK)
        {
            // TODO(Michal/Adam): Implement some diagnostics
            ROS_INFO("Received automatic mode request");
            usleep(WAIT_FOR_TTCONTROL);
            CommHandlerControl.vConfirmAdvantechReadiness();
        }
        // if automatic state request error frame received in u8ParseCanFrames()
        // method
        if ((u8Status&AUTOMATIC_STATE_REQUEST_ERROR_MASK) ==
             AUTOMATIC_STATE_REQUEST_ERROR_MASK)
        {
            ROS_ERROR("Automatic state request error: %d",
                      CommHandlerControl.iGetErrorCode());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
