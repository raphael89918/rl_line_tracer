#include "move_cam/move_cam.h"

Move::Move(ros::NodeHandle &nh)
    : motor1(1, 0, "/dev/ttyUSB0")
    , kbin(0)
    , nh_(nh)
{
    ROS_INFO("\n MOVE CLASS CONSTRUCTED\n");
}

Move::~Move()
{
    ROS_INFO("\n MOVE CLASS DECONSTRUCTED\n");
}

void Move::setposition()
{
   // nh.getParam("port",this->port);
  //  motor1(1, 0,port);
    position = 150;
    motor1.setServoState(ON);
    motor1.setSpeed(30);
    motor1.setPosition(position);
    motor1.waitForIdle();
    cout << "motor1 angle:" << position << endl;
    motor1.setServoState(OFF);
}
void Move::run()
{
    position = 150;
    motor1.setServoState(ON);
    while (kbin != 27) {
        cout << "enter '1' to turn backword,'2'to turn forward " << endl;
        kbin = getch();
        if (kbin == 49) {      // keyboard 1
            position = position +10 ; // open
            motor1.setSpeed(15);
            motor1.setPosition(position);
            cout << "motor1 angle:" << position << endl;
        }
        if (kbin == 50) {      // keyboard 2
            position = position -10; // close
            motor1.setSpeed(15);
            motor1.setPosition(position);
            cout << "motor1 angle:" << position << endl;
        }
        motor1.waitForIdle();
    }
    motor1.setServoState(OFF);
}

