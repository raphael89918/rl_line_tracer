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

    inline int get_execute_rate();
    inline int get_max_iteration();

    void init();  //initialize the planner
    void start(); //start the planner
    void suspend();
    void execute();
    void plan(); //plan offline collect

    inline void stop_wheel();

    void get_state_reward();
    driving_action get_remote_action();
    void set_action(const driving_action &action);

private:
    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Publisher m_pub_action;
    ros::Subscriber m_sub_action;
    ros::Rate m_execute_rate;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_sub_msg;
    reinforcement_learning_planner::action m_action_pub_msg;

    RL_handler m_rl_handler;
    PlannerState m_planner_state;

    inline void state_callback(const reinforcement_learning_planner::state &msg);
    inline void reward_callback(const reinforcement_learning_planner::reward &msg);
    inline void action_callback(const reinforcement_learning_planner::action &msg);

    bool m_exit;
    int m_max_iteration;
};