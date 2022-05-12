#include "ltproc/ltproc.h"
#include "cam/cam.h"
ltproc::ltproc(ros::NodeHandle &nh)
    : corner(false)
    , line(false)
    , lt_lane_width(60)
    , lt_prune_dis(30)
    , d(5)
    , sigmaColor(150)
    , sigmaSpace(150)
    , th1(50)
    , th2(150)
    , dilate_iter(1)
    , erode_iter(1)
    , image_transport_(nh)
      , nh_(nh)

{
    ROS_INFO("\n Ltproc Class Constructed\n");
    offset_pub = nh.advertise<reinforcement_learning_planner::state>("/state", 5);
}

ltproc::~ltproc()
{
    ROS_INFO("\n Ltproc Class Destructed\n");
}
bool ltproc::Capture(Mat input)
{
    if(!input.empty())
        lt_frame = input;
    return true;
}
void ltproc::set_cam(int camIndex)
{
    lt_camIndex = camIndex;
    lt_cam = cv::VideoCapture(lt_camIndex);
}

void ltproc::set_window(int width, int height)
{
    rsl_width = width;
    rsl_height = height;
    lt_cam.set(cv::CAP_PROP_FRAME_WIDTH, rsl_width);
    lt_cam.set(cv::CAP_PROP_FRAME_HEIGHT, rsl_height);
    for (int i = 0; i < 7; i++) {
        mask[i].create(rsl_height, rsl_width, CV_8U);
        mask[i] = cv::Scalar(0);
        cv::ellipse(mask[i], cv::Point(mask[i].cols / 2, mask[i].rows), cv::Size(320, 480 - (int)(65 * i)), 0, 0, 360, cv::Scalar(255), 1, 8, 0);
    }
}

void ltproc::resize_window(int width, int height)
{
    rsl_width = width;
    rsl_height = height;

    resize(lt_frame,lt_frame,Size(rsl_width,rsl_height));
    for (int i = 0; i < 7; i++) {
        mask[i].create(rsl_height, rsl_width, CV_8U);
        mask[i] = cv::Scalar(0);
        cv::ellipse(mask[i], cv::Point(mask[i].cols / 2, mask[i].rows), cv::Size(320, 480 - (int)(70 * i)), 0, 0, 360, cv::Scalar(255), 1, 8, 0);
    }
}

void ltproc::set_lane_width(int width) { lt_lane_width = width; }
void ltproc::set_prune_distance(int prune_dis) { lt_prune_dis = prune_dis; }

void ltproc::find_contour()
{
    if(!lt_frame.empty())
    {
        cv::erode(lt_frame, ero, cv::Mat(), cv::Point(-1, -1), erode_iter);
        cv::bilateralFilter(ero, blur, d, sigmaColor, sigmaSpace);
        //   cv::cvtColor(blur, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(blur, can, th1, th2, 3);
        cv::dilate(can, dil, cv::Mat(), cv::Point(-1, -1), dilate_iter);
    }
}

void ltproc::lt_proc()
{
    find_contour();
    find_side_point();
    prune_by_distance();
    find_offset();
    find_corner();
}

void ltproc::find_side_point()
{
    if(!dil.empty())
    {
        if (!sidePt.empty())
            sidePt.clear();
        lt_coor tmp;
        cv::Mat buf;
        for (int i = 0; i < 7; i++) {
            cv::bitwise_and(dil, mask[i], buf);

            for (int j = 0; j < dil.rows; j++) {
                for (int k = 0; k < dil.cols; k++) {
                    if (buf.at<uchar>(j, k) > 0) {
                        tmp.x = k;
                        tmp.y = j;
                        tmp.num = i;
                        sidePt.push_back(tmp);
                    }
                }
            }
        }
    }
}
void ltproc::prune_by_distance()
{
    bool pruned = true;
    while (pruned) {
        pruned = false;
        for (int j = 0; j < (int)sidePt.size(); j++) {
            if (j + 1 != (int)sidePt.size()) {
                if (sidePt[j].num == sidePt[j + 1].num) {
                    if (calc_dis(sidePt[j], sidePt[j + 1]) < lt_prune_dis) {
                        sidePt.erase(sidePt.begin() + j);
                        pruned = true;
                    }
                }
            }
        }
    }

    /*while (1) {
      pruned = false;
      for (int i = 0; i < (int)sidePt.size(); i++) {
      for (int j = i + 1; j < (int)sidePt.size() - 1; j++) {
      if (calc_dis(sidePt[i], sidePt[j]) < lt_prune_dis) {
      sidePt.erase(sidePt.begin() + j);
      pruned = true;
      }
      }
      for (int j = i - 1; j >= 0; j--) {
      if (calc_dis(sidePt[i], sidePt[j]) < lt_prune_dis) {
      sidePt.erase(sidePt.begin() + j);
      pruned = true;
      }
      }
      }
      if (!pruned)
      break;
      }*/

    std::vector<lt_coor> new_sidePt;

    for (int i = 0; i < 7; i++) {
        int ptA = 0, ptB = 0;
        float widthDiff, currentDiff;
        std::vector<lt_coor> tmp;
        std::vector<int> ptbuf;
        bool first_time = true;

        for (int j = 0; j < (int)sidePt.size(); j++) {
            if (sidePt[j].num == i) {
                tmp.push_back(sidePt[j]);
                ptbuf.push_back(j);
            }
        }

        for (int j = 0; j < (int)tmp.size(); j++) {
            for (int k = 0; k < (int)tmp.size(); k++) {
                widthDiff = fabs(calc_dis(tmp[j], tmp[k]) - lt_lane_width);
                if (j != k && (first_time || widthDiff < currentDiff)) {
                    currentDiff = widthDiff;
                    ptA = ptbuf[j];
                    ptB = ptbuf[k];
                    first_time = false;
                }
            }
        }
        if (!sidePt.empty() && ptA != ptB) {
            new_sidePt.push_back(sidePt[ptA]);
            new_sidePt.push_back(sidePt[ptB]);
        }
    }
    if (!sidePt.empty()) {
        sidePt.clear();
    }
    sidePt = new_sidePt;
    if (!new_sidePt.empty()) {
        new_sidePt.clear();
    }
}

void ltproc::find_offset()
{
    reinforcement_learning_planner::state msg;
    if (!midPt.empty())
        midPt.clear();

    lt_coor tmp;
    cam_offset = 0;

    if (!sidePt.empty()) {
        for (int i = 0; i < (int)sidePt.size() / 2; i++) {
            tmp.x = (sidePt[2 * i].x + sidePt[2 * i + 1].x) / 2;
            tmp.y = (sidePt[2 * i].y + sidePt[2 * i + 1].y) / 2;
            tmp.num = sidePt[2 * i].num;
            midPt.push_back(tmp);
        }
        if ((int)midPt.size() >= 6) {
            line = true;
        } else {
            line = false;
        }
    } else {
        line = false;
    }
    if (line) {
        for (int i = 0; i < (int)midPt.size(); i++) {
            cam_offset += ((midPt[i].x - 320) / 2) / 100.0 * weight[midPt[i].num];
        }
    }

  // if((cam_offset < 60)&& (cam_offset >-60)&&(cam_offset!=0.0))

        if((cam_offset>0)&&(cam_offset<=10))
            cam_offset =1;
        else if((cam_offset>10)&&(cam_offset<=20))
            cam_offset =2;
        else if((cam_offset>20)&&(cam_offset<=30))
            cam_offset =3;
        else if((cam_offset>30)&&(cam_offset<=40))
            cam_offset =4;
        else if((cam_offset>40)&&(cam_offset<=50))
            cam_offset =5;
        else if((cam_offset>50)&&(cam_offset<=60))
            cam_offset =6;
        else if((cam_offset>=-10)&&(cam_offset<0))
            cam_offset =-1;
        else if((cam_offset>=-20)&&(cam_offset<-10))
            cam_offset =-2;
        else if((cam_offset>=-30)&&(cam_offset<-20))
            cam_offset =-3;
        else if((cam_offset>=-40)&&(cam_offset<-30))
            cam_offset =-4;
        else if((cam_offset>=-50)&&(cam_offset<-40))
            cam_offset =-5;
        else if((cam_offset>=-60)&&(cam_offset<-50))
            cam_offset =-6;
        else
            cam_offset =0;
        msg.offset = cam_offset;
        msg.special_case = 0;

    offset_pub.publish(msg);

    sidept = sidePt;

    if (!sidePt.empty())
        sidePt.clear();
}

void ltproc::find_corner()
{
    int threshold = 5;
    int count = 0;

    if (line) {
        for (int i = 1; i < (int)midPt.size(); i++) {
            if (abs(midPt[i - 1].y - midPt[i].y) < threshold) {
                count += 1;
                if (count >= 3)
                    corner = true;
            } else
                count = 0;
        }
    } else
        corner = false;
}

    void ltproc::show_result(int x,int y)
{   if(!lt_frame.empty())
    {
        cv::Mat result(rsl_height, rsl_width, CV_8UC3);
        result = cv::Scalar(0);
        for (int i = 0; i < (int)sidept.size(); i++) {
            cv::circle(result, cv::Point(sidePt[i].x, sidept[i].y), 3, cv::Scalar(0, 0, 255), 5);
        }
        std::cout << "side point's size:" << sidept.size() << std::endl;
        std::cout << "offset:" << cam_offset << std::endl;
        cv::namedWindow("result");
        cv::resizeWindow("result",x,y);
        cv::namedWindow("origin");
        cv::resizeWindow("origin",x,y);
        cv::resize(result,result,Size(x,y));
        cv::resize(lt_frame,lt_frame,Size(x,y));
        cv::imshow("result", result);
        cv::imshow("origin", lt_frame);
    }
}

void ltproc::show_gray_img() { cv::imshow("gray", gray); }
void ltproc::show_canny_img() { cv::imshow("canny", can); }
void ltproc::show_erode_img() { cv::imshow("erode", ero); }
void ltproc::show_blur_img() { cv::imshow("blur", blur); }
void ltproc::show_dilate_img() { cv::imshow("dilate", dil); }
float ltproc::calc_dis(lt_coor a, lt_coor b)
{
    float distance = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return distance;
}

