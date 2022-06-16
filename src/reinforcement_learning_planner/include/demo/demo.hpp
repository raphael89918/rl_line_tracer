#pragma once

#include "conio.h"

#include <string>
#include <vector>

#include "ros/ros.h"
#include "planner/planner.hpp"

#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"

#include "rl_handler/rl_handler.hpp"

enum class DemoChoice
{
    ONLINE,
    OFFLINE
};

class Demo
{
public:
    Demo();
    Demo(ros::NodeHandle &nh, DemoChoice demo_choice);
    ~Demo();

    inline int get_execute_rate();

    void init();  //initialize the planner
    void start(); //start the planner
    void suspend();
    void execute();
    void plan(); //plan offline collect

    inline void stop_wheel();

    void get_state();
    void set_action(const driving_action &action);

    std::string choice_to_filepath();

private:
    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_state;
    ros::Publisher m_pub_action;
    ros::Rate m_execute_rate;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::action m_action_pub_msg;

    DemoChoice m_demo_choice;
    RL_handler m_rl_handler;
    PlannerState m_planner_state;

    inline void state_callback(const reinforcement_learning_planner::state &msg);
    inline void action_callback(const reinforcement_learning_planner::action &msg);

    bool m_exit;
};