#pragma once

#include "conio.h"

#include <string>
#include <vector>

#include "ros/ros.h"
#include "reinforcement_learning_planner/action.h"
#include "reinforcement_learning_planner/state.h"
#include "reinforcement_learning_planner/reward.h"

#include "planner/planner.hpp"
#include "rl_handler/rl_handler.hpp"

class OnlineTraining
{

    using rl_state = relearn::state<semantic_line_state>;
    using rl_action = relearn::action<driving_action>;
    using rl_episode = relearn::link<rl_state, rl_action>;

public:
    OnlineTraining();
    OnlineTraining(ros::NodeHandle &nh);
    ~OnlineTraining();

    int get_execute_rate();

    void init();    //initialize the planner
    void start();   //start the planner
    void suspend(); //suspend the planner
    void execute();

    void stop_wheel();
    void rotate_wheel();
    void revert_wheel(std::reverse_iterator<std::deque<rl_episode>::iterator> it);

    void plan(); //plan online training

    void get_state_reward();
    void set_action();

    void out_of_bounds_trap();

private:
    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Publisher m_pub_action;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_msg;

    RL_handler m_rl_handler;
    PlannerState m_planner_state;

    void state_callback(const reinforcement_learning_planner::state &msg);
    void reward_callback(const reinforcement_learning_planner::reward &msg);

    int m_execute_rate;

    bool m_exit;
};