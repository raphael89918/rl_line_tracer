#ifndef __LTPROC_H__
#define __LTPROC_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <ros/ros.h>
#include "vector"
#include <chrono>
#include "std_msgs/Int8.h"
#include <reinforcement_learning_planner/state.h>
#include <iomanip>

using namespace std;
using namespace cv;

class lt_coor {
    public:
        int x;
        int y;
        int num;
        unsigned int peer;
};

class ltproc {
    public:
        void set_cam(int camIndex);
        void resize_window(int height, int width);
        void set_window(int height, int width);
        void set_lane_width(int width);
        void set_prune_distance(int prune_dis);
        void lt_proc();
        bool corner;
        bool line;
        bool Capture(Mat input);
        void show_result(int x,int y);
        void show_gray_img();
        void show_canny_img();
        void show_erode_img();
        void show_blur_img();
        void show_dilate_img();
        vector<lt_coor> sidept;
        Mat lt_frame;
        float calc_dis(lt_coor a, lt_coor b);
        float cam_offset;
        float first_point;
        float last_point;
        ltproc(ros::NodeHandle &nh);
        ~ltproc();

    private:
        ros::NodeHandle nh_;
        ros::Publisher offset_pub;
         image_transport::ImageTransport image_transport_;
        Mat mask[7];
        float weight[7] = { 1, 1, 5, 5, 5, 10, 10 };
        VideoCapture lt_cam;
        int lt_lane_width;
        int lt_prune_dis;
        int rsl_width, rsl_height;
        vector<lt_coor> sidePt;
        vector<lt_coor> midPt;
        void find_contour();
        void find_side_point();
        void prune_by_distance();
        void find_offset();
        void find_corner();
        int lt_camIndex;
        Mat gray;
        Mat can;
        Mat ero;
        Mat blur;
        Mat dil;
        Mat pic;
        int d;
        int sigmaColor;
        int sigmaSpace;
        int th1;
        int th2;
        int dilate_iter;
        int erode_iter;
};

#endif

