#include "twist_to_pwm/twist_to_pwm.hpp"
twist_transform::twist_transform(const ros::NodeHandle &nh)
    : m_nh(nh), t_nh(nh)
{
    ROS_INFO("class twist_transform has been constructed");
}
twist_transform::~twist_transform()
{
    ROS_INFO("class twist_transform has been destructed");
}
void twist_transform::start()
{
    ROS_INFO("Starting to set up pub and sub");
    m_pub = m_nh.advertise<wheel_controller::motor>("/wheel/motor", 1);
    m_sub = t_nh.subscribe("/cmd_vel", 1, &twist_transform::callback, this);
    ROS_INFO("Initialzing motor parameter");

}


void twist_transform::callback(const geometry_msgs::Twist &cmd_vel)
{
    double vel_x = cmd_vel.linear.x;
    double vel_th = cmd_vel.angular.z;
    right_vel=vel_x-vel_th*ROBOT_WIDTH/2000;
    left_vel=vel_x+vel_th*ROBOT_WIDTH/2000;
    setSpeed(LEFT,left_vel);
    setSpeed(RIGHT,right_vel);
}

void twist_transform::setSpeed(char channel, double vel)
{
    vel = vel*255;
    if(vel>0)
    {
        if(vel>255)
        {
            spd=255;
            dir=1;
        }
        else
        {
            dir=1;
            spd=(unsigned char)vel;
        }
    }
    if(vel<0)
    {
        if(vel<-255)
        {
            spd=255;
            dir=0;
        }
        else
        {
            dir=0;
            spd=(unsigned char)(-vel);
        }
    }
    if(vel==0)
    {
        dir=0;
        spd=0;
    }
    if(channel== LEFT)
    {
        m_msg.FL = spd;
        m_msg.BL = spd;
        m_msg.FL_DIR = dir;
        m_msg.BL_DIR = dir;
    }else if(channel== RIGHT)
    {
        m_msg.FR = spd;
        m_msg.BR = spd;
        m_msg.FR_DIR = dir;
        m_msg.BR_DIR = dir;
    }
}


void twist_transform::execute()
{
    ros::Rate loop_rate(10);
    ros::spinOnce();
    m_pub.publish(m_msg);
    loop_rate.sleep();
}
