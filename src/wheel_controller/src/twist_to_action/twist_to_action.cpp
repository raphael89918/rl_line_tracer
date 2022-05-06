#include "twist_to_action/twist_to_action.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_action_node");
    ros::NodeHandle nh;

    action_transform action_transform(nh);
    action_transform.start();
    while(ros::ok())
    {
        action_transform.execute();
    }
}
