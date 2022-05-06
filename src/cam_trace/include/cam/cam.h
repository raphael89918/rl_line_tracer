#ifndef __CAM_H__
#define __CAM_H__

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;


class Cam{
    private:
        Mat final;
        ros::NodeHandle nh_;

    public:
        Cam(ros::NodeHandle &nh);
        ~Cam();
        void Show();
        Mat frame;
        void Callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
