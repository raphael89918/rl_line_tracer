#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
using namespace std;

class Teleop
{
public:
    Teleop();

private:
    /* data */
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);
    ros::NodeHandle n;
    ros::Subscriber sub ;
    ros::Publisher pub ;
    double vlinear,vangular;
    int axis_ang,axis_lin,ton;
};

Teleop::Teleop()
{
    n.param<int>("axis_linear",axis_lin,1);
    n.param<int>("axis_angular",axis_ang,2);
    n.param<double>("vel_linear",vlinear,0.2);
    n.param<double>("vel_angular",vangular,1);
    n.param<int>("button",ton,5);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    sub = n.subscribe<sensor_msgs::Joy>("joy",1,&Teleop::callback,this);
}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    geometry_msgs::Twist v;
    ros::Rate loop_rate(100);
    if(Joy->buttons[ton])
    {
         if(Joy->axes[axis_lin] > 0 && Joy->axes[axis_ang] == 0)
         {
             v.linear.x = 0.2;
             v.angular.z = 0;
         }
         if(Joy->axes[axis_lin] > 0 && Joy->axes[axis_ang] != 0)
         {
             v.linear.x = 0.2;
             if(Joy -> axes[axis_ang] > 0)
             {
                 v.angular.z = -1;
             }
             if(Joy -> axes[axis_ang] < 0)
             {
                 v.angular.z = 1;
             }
         }
         if(Joy->axes[axis_lin] == 0 && Joy->axes[axis_ang] == 0)
         {
             v.linear.x = 0;
             v.angular.z = 0;
         }


     //v.linear.x =(Joy->axes[axis_lin])*vlinear;
     //v.angular.z = - (Joy->axes[axis_ang])*vangular;
     ROS_INFO("linear:%.3lf   angular:%.3lf",v.linear.x,v.angular.z);
     pub.publish(v);
     loop_rate.sleep();
     }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "joy");
    Teleop telelog;
    ros::spin();
    return 0;
}
