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

    inline int get_execute_rate();
    inline int get_max_iteration();

    void init();    //initialize the planner
    void start();   //start the planner
    void suspend(); //suspend the planner
    void execute();

    inline void stop_wheel();
    inline void rotate_wheel();
    inline void revert_wheel();

    void plan(); //plan online training

    void get_state_reward();
    void set_action(const driving_action &action);

    void out_of_bounds_trap();

private:
    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_state;
    ros::Subscriber m_sub_reward;
    ros::Publisher m_pub_action;
    ros::Rate m_execute_rate;

    reinforcement_learning_planner::state m_state_msg;
    reinforcement_learning_planner::reward m_reward_msg;
    reinforcement_learning_planner::action m_action_msg;

    RL_handler m_rl_handler;
    PlannerState m_planner_state;


    inline void state_callback(const reinforcement_learning_planner::state &msg);
    inline void reward_callback(const reinforcement_learning_planner::reward &msg);

    bool m_exit;
    int m_max_iteration;
};