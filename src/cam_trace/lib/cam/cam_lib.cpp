#include "cam/cam.h"

Cam::Cam(ros::NodeHandle &nh)
    :nh_(nh),cap("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_7CB2D17F-video-index0"
            )
{
    if(!cap.isOpened())
        ROS_ERROR("\n camera not opened\n");
    ROS_INFO("\n Cam Class Constrcted\n");

}

Cam::~Cam()
{
    ROS_INFO("\n Cam Class Destrcted\n");
}

bool Cam::Capture()
{
    cap >> frame;
    //imshow ("frame",frame);
    GaussianBlur(frame,frame,cv::Size(5,5),0,0);
    cvtColor(frame,frame,CV_RGB2GRAY);
     Mat element = getStructuringElement(MORPH_RECT, Size(18, 18));
    morphologyEx(frame,frame, MORPH_CLOSE, element);
    //imshow ("gauss",frame);
    return true;
}
/*
void Cam::Callback(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("\n in Callback \n");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("CV BRIDGE EXCEPTION: %s",e.what());
        return ;
    }
    frame =cv_ptr->image.clone();
    if(frame.empty())
    {
        ROS_ERROR("THE IMAGE IS EMPTY");
    }
    waitKey(10);
}
*/
