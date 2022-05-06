#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "wheel_controller/motor.h"

#define ROBOT_WIDTH 300
#define LEFT 0
#define RIGHT 1

enum WHEEL_POSITION : uint8_t
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3
};

enum DIRECTION : bool
{
    FORWARD = false,
    BACKWARD = true,
};

struct wheel_data
{
    uint8_t pwm;
    DIRECTION direction;
};
class twist_transform
{
public:
    twist_transform(const ros::NodeHandle &nh);
    ~twist_transform();

    void start();

    void callback(const geometry_msgs::Twist &msg);
    void execute();
private:
    ros::NodeHandle m_nh;
    ros::NodeHandle t_nh;

    ros::Publisher m_pub;

    wheel_controller::motor m_msg;

    ros::Subscriber m_sub;

    double right_vel, left_vel;
    void setSpeed(char channel, double vel);
    unsigned char dir;
    unsigned char spd;
    unsigned char leftSpd;
    unsigned char leftDir;
    unsigned char rightSpd;
    unsigned char rightDir;

};
