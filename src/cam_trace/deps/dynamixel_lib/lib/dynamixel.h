#ifndef __DYNAMIXEL_H__
#define __DYNAMIXEL_H__

#include <iostream>

#include "protocolHandler.h"

enum STATE { OFF, ON };

class Motor {
   public:
    enum class PROTOCOL1 {
        OPEN_CONTROL = 24,
        MOVE = 30,
        SPEED = 32,
        PRESENT_POSITION = 36,
        PRESENT_SPEED = 38,
        SETTING_LOCK = 47,
        MOVING = 46,
        LOAD = 40
    };

    enum class PROTOCOL2 {
        OPEN_CONTROL = 64,
        MOVE = 116,
        SPEED = 100,
        PRESENT_POSITION = 132,
        PRESENT_SPEED = 128,
        MOVING = 122,
        GOAL_VELOCITY = 104,
        GOAL_PWM = 100
    };

    // use:Motor motor1(1,1,/dev/ttyUSB0);
    Motor(int protocolIn, int IDIn, std::string portName, bool show = true);
    ~Motor();

    int setServoState(STATE state);
    int setSpeed(int speed);
    int setSpeedOrg(int speed);
    int setPosition(int position);
    int checkPosition();
    bool isMoving();
    int setGoalVelocity(int val);
    int setGoalPWM(int val);
    void waitForIdle();

    int reboot();

   private:
    int protocol;
    int ID;

    ProtocolHandler ctx;

    int destPosition;
};

#endif  // DYNAMIXEL_H
