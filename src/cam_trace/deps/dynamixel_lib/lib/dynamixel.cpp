#include "dynamixel.h"

#include <iostream>

using namespace std;

Motor::Motor(int protocolIn, int IDIn, string portName, bool show)
    : ctx(portName, show) {
    this->protocol = protocolIn;
    this->ID = IDIn;
    this->destPosition = 0;
    // this->ctx.reboot(this->protocol, this->ID);
    this->setServoState(ON);

    cout << " - ID          : " << this->ID << endl << endl;
}

Motor::~Motor() {}

int Motor::setServoState(STATE state) {
    int result = -1;

    if (this->protocol == 1) {
        this->ctx.write_bits(this->protocol, this->ID,
                             (int)PROTOCOL1::OPEN_CONTROL, (int)state);
        result = this->ctx.read_bits(this->protocol, this->ID,
                                     (int)PROTOCOL1::OPEN_CONTROL);
        if (result != (int)state) {
            cerr << "setServoState error!!" << endl;
            return -1;
        }
    } else if (this->protocol == 2) {
        this->ctx.write_bits(this->protocol, this->ID,
                             (int)PROTOCOL2::OPEN_CONTROL, (int)state);
        result = this->ctx.read_bits(this->protocol, this->ID,
                                     (int)PROTOCOL2::OPEN_CONTROL);
        if (result != (int)state) {
            cerr << "setServoState error!!" << endl;
            return -1;
        }
    } else {
        cerr << "error status!!" << endl;
        return -1;
    }
    return 0;
}

int Motor::setSpeed(int speed) {
    int result = -1;

    if (this->protocol == 1) {
        ctx.write_words(this->protocol, this->ID, (int)PROTOCOL1::SPEED,
                        (int)speed * 1023 / 100);
        result = this->ctx.read_words(this->protocol, this->ID,
                                      (int)PROTOCOL1::SPEED);
        if (result != (int)speed * 1023 / 100) {
            cerr << "setSpeed error!!" << endl;
            return -1;
        }
    } else if (this->protocol == 2) {
        ctx.write_dwords(this->protocol, this->ID, (int)PROTOCOL2::SPEED,
                         speed * 885 / 100);
        result = this->ctx.read_dwords(this->protocol, this->ID,
                                       (int)PROTOCOL2::SPEED);
        if (result != (int)speed * 885 / 100) {
            cerr << "setSpeed error!!" << endl;
            return -1;
        }
    } else {
        cerr << "error status!!" << endl;
        return -1;
    }

    return 0;
}

int Motor::setPosition(int position) {
    int result = -1;

    if (this->protocol == 1) {
        if (position < 0 || position > 300) {
            cout << "degree error!" << endl;
            return -2;
        }
        ctx.write_words(this->protocol, this->ID, (int)PROTOCOL1::MOVE,
                        (int)position * 1023 / 300);
        result = this->ctx.read_words(this->protocol, this->ID,
                                      (int)PROTOCOL1::MOVE);
        if (result != (int)position * 1023 / 300) {
            cerr << "setPosition error!!" << endl;
            return -1;
        }
    } else if (this->protocol == 2) {
        if (position < 0 || position > 360) {
            cout << "degree error!" << endl;
            return -2;
        }
        ctx.write_dwords(this->protocol, this->ID, (int)PROTOCOL2::MOVE,
                         position * 4095 / 360);
        result = this->ctx.read_dwords(this->protocol, this->ID,
                                       (int)PROTOCOL2::MOVE);
        if (result != (int)position * 4095 / 360) {
            cerr << "setPosition error!!" << endl;
            return -1;
        }
    } else {
        cerr << "error status!!" << endl;
        return -1;
    }

    return 0;
}

int Motor::checkPosition() {
    int result = -1;

    if (this->protocol == 1) {
        result = ctx.read_words(this->protocol, this->ID,
                                (int)PROTOCOL1::PRESENT_POSITION) *
                 300 / 1023;
    }

    else if (this->protocol == 2) {
        result = ctx.read_dwords(this->protocol, this->ID,
                                 (int)PROTOCOL2::PRESENT_POSITION) *
                 360 / 4095;
    }

    else {
        cerr << "error status!!" << endl;
        return -1;
    }

    return (destPosition = result);
}

int Motor::setGoalVelocity(int val) {
    int result = -1;

    if (this->protocol == 1) {
        cerr << "protocol1 cannot setGoalVelocity" << endl;
    } else if (this->protocol == 2) {
        ctx.write_dwords(this->protocol, this->ID,
                         (int)PROTOCOL2::GOAL_VELOCITY, val);
        result = this->ctx.read_dwords(this->protocol, this->ID,
                                       (int)PROTOCOL2::GOAL_VELOCITY);
        if (result != val) {
            cerr << "setGoalVelocity error!!" << endl;
            return -1;
        }
    } else {
        cerr << "error status!!" << endl;
        return -1;
    }

    return 0;
}

int Motor::setGoalPWM(int val) {
    int result = -1;

    if (this->protocol == 1) {
        cerr << "protocol1 cannot setGoalPWM" << endl;
    } else if (this->protocol == 2) {
        ctx.write_dwords(this->protocol, this->ID, (int)PROTOCOL2::GOAL_PWM,
                         val);
        result = this->ctx.read_dwords(this->protocol, this->ID,
                                       (int)PROTOCOL2::GOAL_PWM);
        if (result != val) {
            cerr << "setGoalPWM error!!" << endl;
            return -1;
        }
    } else {
        cerr << "error status!!" << endl;
        return -1;
    }

    return 0;
}

bool Motor::isMoving() {
    int result = 1;

    if (this->protocol == 1) {
        result =
            ctx.read_bits(this->protocol, this->ID, (int)PROTOCOL1::MOVING);
    }

    else if (this->protocol == 2) {
        result =
            ctx.read_bits(this->protocol, this->ID, (int)PROTOCOL2::MOVING);
    }

    return result;
}

int Motor::reboot() { return this->ctx.reboot(2, this->ID); }

void Motor::waitForIdle() {
    while (isMoving()) {
    }
}
