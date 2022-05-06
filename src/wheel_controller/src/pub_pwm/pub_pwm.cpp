#include "ros/ros.h"
#include "wheel_controller/motor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<wheel_controller::motor>("/wheel/motor",1);
    wheel_controller::motor msg;
    while(ros::ok())
    {
        msg.FL = 50;
        msg.FR = 50;
        msg.FL_DIR = 0;
        msg.FR_DIR = 1;
        pub.publish(msg);
    }
    return 0;
}
