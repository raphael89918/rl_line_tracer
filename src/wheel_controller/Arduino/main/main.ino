#include <ros.h>

#include "motor_controller.hpp"

#include "qtr_msg.hpp"

ros::NodeHandle nh;

MotorController motor_controller(nh);

MyQTR MyQTR(nh);

void setup()
{
    nh.initNode();

    nh.subscribe(motor_controller.m_sub);
    nh.advertise(MyQTR.pub);

    motor_controller.set_motor_driver(MD01, 2);

    motor_controller.set_pin(FL,
                             7, 6, 5);

    motor_controller.set_pin(FR,
                             4,3,5);

    motor_controller.subscribe();

    MyQTR.qtr.setSensorPins((const uint8_t[]){31, 33, 35, 37, 30, 32, 34, 36},8);

    MyQTR.setQTR();
}

void loop()
{
    //motor_controller.spin_once();
    MyQTR.pubQTR();
    nh.spinOnce();
    delay(100);
}
