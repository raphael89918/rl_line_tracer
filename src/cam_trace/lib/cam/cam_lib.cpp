#include "cam/cam.h"

Cam::Cam(ros::NodeHandle &nh)
    :nh_(nh)
{
    ROS_INFO("\n Cam Class Constrcted\n");

}

Cam::~Cam()
{
    ROS_INFO("\n Cam Class Destrcted\n");
}

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

