#ifndef __MOVE_CAM_H__
#define __MOVE_CAM_H__

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "dynamixel.h"
#include "control/control.h"
using namespace std;

class Move
{
    public:
        void run();
        void setposition();
        Move(ros::NodeHandle &nh);
        ~Move();
    private:
        ros::NodeHandle nh_;
        Motor motor1;
        char kbin;
        int position;
        string port;
};

#endif
