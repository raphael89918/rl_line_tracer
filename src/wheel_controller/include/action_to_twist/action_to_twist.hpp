#include "ros/ros.h"
#include "reinforcement_learning_planner/action.h"
#include "geometry_msgs/Twist.h"

class action_transform
{
public:
    action_transform(const ros::NodeHandle &nh);

    void callback(const reinforcement_learning_planner::action &msg);
    void start();
    void execute();
    void translate(double linear, double angular);

private:
    ros::NodeHandle t_nh;
    ros::NodeHandle a_nh;
    ros::Publisher t_pub;

    geometry_msgs::Twist t_msg;

    ros::Subscriber a_sub;
};

