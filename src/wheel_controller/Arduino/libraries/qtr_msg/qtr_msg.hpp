#include "ros.h"
#include <QTRSensors.h>
#include <std_msgs/UInt16MultiArray.h>

class MyQTR
{
private:
     std_msgs::UInt16MultiArray msg;
     ros::NodeHandle nh;

     uint16_t sensorValues[8];
public:

     QTRSensors qtr;

     MyQTR(ros::NodeHandle &nh);

     ros::Publisher pub;
     void setQTR();
     void pubQTR();
};
