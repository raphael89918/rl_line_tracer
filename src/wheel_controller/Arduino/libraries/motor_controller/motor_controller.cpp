#include "motor_controller.hpp"

MotorController::MotorController(ros::NodeHandle &nh)
    : m_motor_flip_rotation{false, false, false, false}, m_driver_id(0), m_driver_num(0),
      m_nh(nh), m_sub("/wheel/motor", &MotorController::motor_callback, this)
{
    m_nh.loginfo("MotorController Constructed");
}

MotorController::~MotorController()
{
    m_nh.loginfo("MotorController Destructed");
}

void MotorController::set_motor_driver(uint8_t MOTOR_DRIVER_type, uint8_t MOTOR_DRIVER_num)
{
    m_driver_id = MOTOR_DRIVER_type;
    m_driver_num = MOTOR_DRIVER_num;
}

void MotorController::set_pin(uint8_t motor_pair,
                              uint8_t ain1, uint8_t ain2, uint8_t pwma,
                              uint8_t bin1, uint8_t bin2, uint8_t pwmb,
                              uint8_t stby)
{
    m_nh.loginfo("Setting motor driver pin, A pin is left motor, B pin is right motor by preset");

    m_motor_driver[motor_pair].pin[AIN1] = ain1;
    m_motor_driver[motor_pair].pin[AIN2] = ain2;
    m_motor_driver[motor_pair].pin[PWMA] = pwma;

    m_motor_driver[motor_pair].pin[BIN1] = bin1;
    m_motor_driver[motor_pair].pin[BIN2] = bin2;
    m_motor_driver[motor_pair].pin[PWMB] = pwmb;

    m_motor_driver[motor_pair].pin[STBY] = stby;

    m_nh.loginfo("Initializing motor driver value");

    m_motor_driver[motor_pair].value[AIN1] = 1;
    m_motor_driver[motor_pair].value[AIN2] = 0;

    m_motor_driver[motor_pair].value[BIN1] = 1;
    m_motor_driver[motor_pair].value[BIN2] = 0;

    m_motor_driver[motor_pair].value[PWMA] = 0;
    m_motor_driver[motor_pair].value[PWMB] = 0;

    m_motor_driver[motor_pair].value[STBY] = HIGH;

    m_nh.loginfo("Initializing motor pinmode");

    for (size_t i = 0; i < 7; ++i)
        pinMode(m_motor_driver[motor_pair].pin[i], OUTPUT);
}

void MotorController::set_pin(uint8_t motor_position,
                              uint8_t dir, uint8_t pwm, uint8_t slp)
{
    m_nh.loginfo("Setting motor driver pin");

    m_motor_driver[motor_position].pin[DIR] = dir;
    m_motor_driver[motor_position].pin[PWM] = pwm;
    m_motor_driver[motor_position].pin[SLP] = slp;

    m_nh.loginfo("Initializing motor driver value");

    m_motor_driver[motor_position].value[DIR] = 0;
    m_motor_driver[motor_position].value[PWM] = 0;
    m_motor_driver[motor_position].value[SLP] = HIGH;

    m_nh.loginfo("Initializing motor pinmode");

    for (size_t i = 0; i < 3; ++i)
        pinMode(m_motor_driver[motor_position].pin[i], OUTPUT);
}

void MotorController::flip_rotation(uint8_t MOTOR_POSITION)
{
    m_motor_flip_rotation[MOTOR_POSITION] = true;
}

void MotorController::subscribe()
{
    m_nh.subscribe(m_sub);
}

void MotorController::motor_callback(const wheel_controller::motor &msg)
{
    update_wheel(msg);
    control_wheel(msg);
    //pwm_log(msg);
}

void MotorController::update_wheel(const wheel_controller::motor &msg)
{
    switch (m_driver_id)
    {

    case TB6612_FNG:

        m_motor_driver[FRONT_PAIR].value[AIN1] = (msg.FL_DIR) ^ (m_motor_flip_rotation[FL]);
        m_motor_driver[FRONT_PAIR].value[AIN2] = (!(msg.FL_DIR)) ^ (m_motor_flip_rotation[FL]);
        m_motor_driver[FRONT_PAIR].value[BIN1] = (msg.FR_DIR) ^ (m_motor_flip_rotation[FR]);
        m_motor_driver[FRONT_PAIR].value[BIN2] = (!(msg.FR_DIR)) ^ (m_motor_flip_rotation[FR]);

        m_motor_driver[BACK_PAIR].value[AIN1] = (msg.BL_DIR) ^ (m_motor_flip_rotation[BL]);
        m_motor_driver[BACK_PAIR].value[AIN2] = (!(msg.BL_DIR)) ^ (m_motor_flip_rotation[BL]);
        m_motor_driver[BACK_PAIR].value[BIN1] = (msg.BR_DIR) ^ (m_motor_flip_rotation[BR]);
        m_motor_driver[BACK_PAIR].value[BIN2] = (!(msg.BR_DIR)) ^ (m_motor_flip_rotation[BR]);

        m_motor_driver[FRONT_PAIR].value[PWMA] = msg.FL;
        m_motor_driver[FRONT_PAIR].value[PWMB] = msg.FR;
        m_motor_driver[BACK_PAIR].value[PWMA] = msg.BL;
        m_motor_driver[BACK_PAIR].value[PWMB] = msg.BR;

        break;

    case MD01:

        m_motor_driver[FL].value[DIR] = (msg.FL_DIR) ^ (m_motor_flip_rotation[FL]);
        m_motor_driver[FR].value[DIR] = (msg.FR_DIR) ^ (m_motor_flip_rotation[FR]);
        m_motor_driver[BL].value[DIR] = (msg.BL_DIR) ^ (m_motor_flip_rotation[BL]);
        m_motor_driver[BR].value[DIR] = (msg.BR_DIR) ^ (m_motor_flip_rotation[BR]);

        m_motor_driver[FL].value[PWM] = msg.FL;
        m_motor_driver[FR].value[PWM] = msg.FR;
        m_motor_driver[BL].value[PWM] = msg.BL;
        m_motor_driver[BR].value[PWM] = msg.BR;

        break;
    }
}

void MotorController::control_wheel(const wheel_controller::motor &msg)
{
    switch (m_driver_id)
    {
    case TB6612_FNG:

        for (size_t i = 0; i < m_driver_num; ++i)
        {
            digitalWrite(m_motor_driver[i].pin[AIN1], m_motor_driver[i].value[AIN1]);
            digitalWrite(m_motor_driver[i].pin[AIN2], m_motor_driver[i].value[AIN2]);
            analogWrite(m_motor_driver[i].pin[PWMA], m_motor_driver[i].value[PWMA]);

            digitalWrite(m_motor_driver[i].pin[BIN1], m_motor_driver[i].value[BIN1]);
            digitalWrite(m_motor_driver[i].pin[BIN2], m_motor_driver[i].value[BIN2]);
            analogWrite(m_motor_driver[i].pin[PWMB], m_motor_driver[i].value[PWMB]);

            digitalWrite(m_motor_driver[i].pin[STBY], m_motor_driver[i].value[STBY]);
        }
        break;

    case MD01:

        for (size_t i = 0; i < m_driver_num; ++i)
        {
            digitalWrite(m_motor_driver[i].pin[DIR], m_motor_driver[i].value[DIR]);
            analogWrite(m_motor_driver[i].pin[PWM], m_motor_driver[i].value[PWM]);
            digitalWrite(m_motor_driver[i].pin[SLP], m_motor_driver[i].value[SLP]);
        }
        break;
    }
}

void MotorController::pwm_log(const wheel_controller::motor &msg)
{
    char fl_char[8];
    char fl_log[16];
//    int fl_num = (m_motor_driver[FRONT_PAIR].value[DIR]) ? -m_motor_driver[FRONT_PAIR].value[PWMA] : m_motor_driver[FRONT_PAIR].value[PWMA];

    int fl_num = (m_motor_driver[FRONT_PAIR].value[DIR]) ? -m_motor_driver[FRONT_PAIR].value[PWM] : m_motor_driver[FRONT_PAIR].value[PWM];


    itoa(fl_num, fl_char, 10);
    strcpy(fl_log, "FL:");
    strcat(fl_log, fl_char);
    m_nh.loginfo(fl_log);
}

void MotorController::spin_once()
{
    m_nh.spinOnce();
}
