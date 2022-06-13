#include "twist_to_action/twist_to_action.hpp"

action_transform::action_transform(const ros::NodeHandle &nh)
    :a_nh(nh), t_nh(nh)
{
    ROS_INFO("class action_transform has been constructed");
}


void action_transform::start()
{
    a_pub = a_nh.advertise<reinforcement_learning_planner::action>("/action",1);
    a_sub = t_nh.subscribe("/cmd_vel",1,&action_transform::callback, this);
}

void action_transform::callback(const geometry_msgs::Twist &msg)
{
    double linear = msg.linear.x;
    double angular = msg.angular.z;
    translate(linear, angular);
}

void action_transform::execute()
{
    ros::Rate loop_rate(10);
    ros::spinOnce();
    a_pub.publish(a_msg);
    loop_rate.sleep();
}

void action_transform::translate(double linear, double angular)
{
    if(linear == 0.2 && angular == 0)
    {
        a_msg.linear_action = 1;
        a_msg.angular_action = 0;
    }
    if(linear == 0 && angular == 1.5)
    {
        a_msg.linear_action = 0;
        a_msg.angular_action = 1;
    }
    if(linear == 0 && angular == -1.5)
    {
        a_msg.linear_action = 0;
        a_msg.angular_action = -1;
    }
    if(linear == 0.2 && angular == 1)
    {
        a_msg.linear_action = 1;
        a_msg.angular_action = 1;
    }
    if(linear == 0.2 && angular == -1)
    {
        a_msg.linear_action = 1;
        a_msg.angular_action = -1;
    }
    if(linear == 0 && angular == 0)
    {
        a_msg.linear_action = 0;
        a_msg.angular_action = 0;
    }
}
