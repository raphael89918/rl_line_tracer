#include "action_to_twist/action_to_twist.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_to_twist_node");
    ros::NodeHandle nh;

    action_transform action_transform(nh);
    action_transform.start();
    while(ros::ok())
    {
        action_transform.execute();
    }
}
