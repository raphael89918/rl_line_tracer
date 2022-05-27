#include "action_to_twist/action_to_twist.hpp"

action_transform::action_transform(const ros::NodeHandle &nh)
    :a_nh(nh), t_nh(nh)
{
    ROS_INFO("class action_transform has been constructed");
}


void action_transform::start()
{
    a_sub = a_nh.subscribe("/wheel/control",1,&action_transform::callback, this);
    t_pub = t_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void action_transform::callback(const reinforcement_learning_planner::action &msg)
{
    double linear = msg.linear_action;
    double angular = msg.angular_action;
    translate(linear, angular);
}

void action_transform::execute()
{
    ros::Rate loop_rate(10);
    ros::spinOnce();
    t_pub.publish(t_msg);
    loop_rate.sleep();
}

void action_transform::translate(double linear, double angular)
{
    if(angular == 0)
    {
        t_msg.angular.z = 0;
    }

    if(angular == 1)
    {
        t_msg.angular.z = 0.25;
    }

    if(angular == 2)
    {
        t_msg.angular.z = 0.75;
    }

    if(angular == -1)
    {
        t_msg.angular.z = -0.25;
    }

    if(angular == -2)
    {
        t_msg.angular.z = -0.75;
    }

    if(linear == 0)
    {
        t_msg.linear.x = 0;
    }

    if(linear == 1)
    {
        t_msg.linear.x = 0.05;
    }

    if(linear == 2)
    {
        t_msg.linear.x = 0.15;
    }
    if(linear == -1)
    {
        t_msg.linear.x = -0.05;
    }
    if(linear == -2)
    {
        t_msg.linear.x = -0.15;
    }
}
