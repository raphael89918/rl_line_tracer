#include "ros/ros.h"
#include "reinforcement_learning_planner/action.h"
#include "geometry_msgs/Twist.h"

class action_transform
{
public:
    action_transform(const ros::NodeHandle &nh);

    void callback(const geometry_msgs::Twist &msg);
    void start();
    void execute();
    void translate(double linear, double angular);

private:
    ros::NodeHandle t_nh;
    ros::NodeHandle a_nh;
    ros::Publisher a_pub;

    reinforcement_learning_planner::action a_msg;

    ros::Subscriber a_sub;

};
