#pragma once

#include "conio.h"

#include <string>
#include <vector>

#include "ros/ros.h"
#include "planner/planner.hpp"

#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

#include "rl_handler/rl_handler.hpp"
class OfflineCollect
{
public:
    OfflineCollect();
    OfflineCollect(ros::NodeHandle &nh);
    ~OfflineCollect();

    void init();  //initialize the planner
    void start(); //start the planner
    void suspend();
    void execute();
    void plan(); //plan offline collect

    void stop_wheel();

    void get_state_reward();
    void get_action();
    void set_action();

private:
    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Subscriber m_sub_action;
    ros::Publisher m_pub_action;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_sub_msg;
    reinforcement_learning_planner::action m_action_pub_msg;

    RL_handler m_rl_handler;
    PlannerState m_planner_state;

    void state_callback(const reinforcement_learning_planner::state::ConstPtr &msg);
    void reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg);

    bool m_exit;
};