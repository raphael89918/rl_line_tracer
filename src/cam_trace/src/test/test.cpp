#include "test/test.hpp"
#include "std_msgs/Float32.h"
void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f]", msg->data);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("ltproc_publisher", 10, chatterCallback);
    ros::spin();
    return 0;
}
