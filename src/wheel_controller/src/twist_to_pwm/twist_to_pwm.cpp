#include "twist_to_pwm/twist_to_pwm.hpp"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_pwm_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    twist_transform twist_transform(nh);
    twist_transform.start();
    while(ros::ok())
    {
        twist_transform.execute();
    }
}
