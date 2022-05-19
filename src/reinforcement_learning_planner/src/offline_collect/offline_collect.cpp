#include "offline_collect/offline_collect.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rl_planner");
    ros::NodeHandle nh;

    OfflineCollect offline_collect(nh);

    offline_collect.start();

    return 0;
}