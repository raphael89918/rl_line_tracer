#include <ros/ros.h>
#include <iostream>
#include <reinforcement_learning_planner/state.h>
#include <geometry_msgs/Twist.h>

class cheat
{
public:
    void callback(const reinforcement_learning_planner::state &state);
    void right();
    void left();
    void straight();
    void execute();
    void start();
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::Twist msg;

    ros::NodeHandle nh;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cheat_node");

    cheat cheat;
    cheat.start();
    while(ros::ok())
    {
        cheat.execute();
    }
}

void cheat::callback(const reinforcement_learning_planner::state &state)
{
    if(state.offset<=9 && state.offset>=2)
    {
        right();
    }
    else if(state.offset>=-9 && state.offset<=-2)
    {
        left();
    }
    else if(state.offset<=1 && state.offset>=-1)
    {
        straight();
    }
    else
    {
        msg.linear.x = 0;
        msg.angular.z = 0;
    }
}

void cheat::start()
{
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub = nh.subscribe("/state", 1, &cheat::callback, this);
}

void cheat::right()
{
    msg.linear.x = 0.2;
    msg.angular.z = 1;
}
void cheat::left()
{
    msg.linear.x = 0.2;
    msg.angular.z = -1;
}
void cheat::straight()
{
    msg.linear.x = 0.2;
    msg.angular.z = 0;
}

void cheat::execute()
{
    ros::Rate loop_rate(20);
    ros::spinOnce();
    pub.publish(msg);
    loop_rate.sleep();
}
