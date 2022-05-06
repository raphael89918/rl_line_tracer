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
    //angular.z = 0
    if(angular == 0)
    {
        a_msg.angular_action = 0;
    }
    //angular.z >0
    if(angular >0 && angular <= 0.5)
    {
        a_msg.angular_action = 1;
    }
    if(angular >0.5 && angular <= 1)
    {
        a_msg.angular_action = 2;
    }
    //angular.z <0
    if(angular <0 && angular >= -0.5)
    {
        a_msg.angular_action = -1;
    }
    if(angular <-0.5 && angular <= -1)
    {
        a_msg.angular_action = -2;
    }

    //linear.x = 0
    if(linear == 0)
    {
        a_msg.linear_action = 0;
    }
    //linear.x >0
    if(linear >0 && linear <= 0.1)
    {
        a_msg.linear_action = 1;
    }
    if(linear>0.1 && linear <=0.2)
    {
        a_msg.linear_action = 2;
    }
}
