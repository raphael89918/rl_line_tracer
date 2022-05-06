#include "qtr_msg.hpp"

MyQTR::MyQTR(ros::NodeHandle &nh)
    :   pub("/pub_qtr", &msg),nh(nh)
{
    msg.data = 0;
}

void MyQTR::setQTR()
{
    msg.data_length = 8;
    msg.data = (uint16_t*)malloc(sizeof(uint16_t)*msg.data_length);
    qtr.setTypeRC();
//    qtr.setSensorPins((const uint8_t[]){24, 26, 28, 30, 32, 34, 36, 38},8);
//    nh.advertise(pub);
}
void MyQTR::pubQTR()
{
    qtr.read(sensorValues);
    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = sensorValues[i];
    }
    pub.publish(&msg);
//    nh.spinOnce();
}
