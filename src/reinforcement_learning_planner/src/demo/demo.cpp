#include "demo/demo.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;

    Demo demo(nh, DemoChoice::ONLINE);

    demo.start();

    return 0;
}