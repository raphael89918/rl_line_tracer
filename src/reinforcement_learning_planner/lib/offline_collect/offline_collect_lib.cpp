#include "offline_collect/offline_collect.hpp"

OfflineCollect::OfflineCollect()
    : m_execute_rate(get_execute_rate())
{
    ROS_INFO("Default OfflineCollect constructed");
}

OfflineCollect::OfflineCollect(ros::NodeHandle &nh)
    : m_nh(nh),
      m_sub_state(nh.subscribe("/state", 1, &OfflineCollect::state_callback, this)),
      m_sub_reward(nh.subscribe("/reward", 1, &OfflineCollect::reward_callback, this)),
      m_pub_action(nh.advertise<reinforcement_learning_planner::action>("/action", 1)),
      m_sub_action(nh.subscribe("/action", 1, &OfflineCollect::action_callback, this)),
      m_execute_rate(get_execute_rate()),
      m_rl_handler("/home/ical/rl_line_tracer/rl_model/offline"),
      m_planner_state(PlannerState::INIT),
      m_exit(false)
{
    ROS_INFO("OfflineCollect constructed");
}

OfflineCollect::~OfflineCollect()
{
    ROS_INFO("OfflineCollect destructed");
}

int OfflineCollect::get_execute_rate()
{
    int execute_rate;
    m_nh.getParam("execute_rate", execute_rate);
    return execute_rate;
}

void OfflineCollect::init()
{
    m_rl_handler.init();
    m_rl_handler.load_model(m_rl_handler.get_recent_filename());
}

void OfflineCollect::start()
{
    while (!m_exit)
    {
        switch (m_planner_state)
        {

        case PlannerState::INIT:
        {
            init();
            m_planner_state = PlannerState::SUSPEND;
            break;
        }
        case PlannerState::SUSPEND:
        {
            suspend();
            break;
        }
        case PlannerState::EXECUTING:
        {
            execute();
            break;
        }
        case PlannerState::TERMINATED:
        {
            m_exit = true;
            break;
        }
        default:
        {
            ROS_INFO("Why are you here?");
            break;
        }
        }
    }
}

void OfflineCollect::suspend()
{
    ros::Rate suspend_rate(2);

    while (true)
    {
        ROS_INFO("Suspending, press s to save, press e to execute, press ESC to exit");
        suspend_rate.sleep();
        bool is_suspend = true;

        if (kbhit())
        {
            switch (tolower(getch()))
            {
            case 's':
            {
                m_rl_handler.save_model(m_rl_handler.get_filename_by_cur_time());
                init();
                ROS_INFO("Saved model, reinitialized");
                is_suspend = false;
                break;
            }
            case 'e':
            {
                ROS_INFO("Execute");
                m_planner_state = PlannerState::EXECUTING;
                is_suspend = false;
                break;
            }
            case 27: // ESC
            {
                m_planner_state = PlannerState::TERMINATED;
                is_suspend = false;
                break;
            }
            }
        }

        if (!is_suspend)
            break;
    }
}

void OfflineCollect::execute()
{
    // initialize first state
    ros::spinOnce();
    semantic_line_state state_trait = {m_state_msg.offset};
    m_rl_handler.set_state(m_reward_msg.offset, state_trait);

    while (true)
    {
        plan();
        bool is_executing = true;

        if (kbhit())
        {
            switch (tolower(getch()))
            {
            case 's':
            {
                stop_wheel();
                m_planner_state = PlannerState::SUSPEND;
                is_executing = false;
                break;
            }
            case 27: // ESC
            {
                m_planner_state = PlannerState::TERMINATED;
                is_executing = false;
                break;
            }
            }
        }
        if (!is_executing)
            break;
    }
}

void OfflineCollect::stop_wheel()
{
    m_action_pub_msg.linear_action = 0;
    m_action_pub_msg.angular_action = 0;
    m_pub_action.publish(m_action_pub_msg);
}

void OfflineCollect::plan()
{
    driving_action remote_action = get_remote_action();
    set_action(remote_action);

    m_execute_rate.sleep();
    ros::spinOnce();
    get_state_reward();

    m_rl_handler.learn();
    m_rl_handler.record_episode();
    m_rl_handler.update_state();
}

void OfflineCollect::state_callback(const reinforcement_learning_planner::state &msg)
{
    m_state_msg = msg;
}

void OfflineCollect::reward_callback(const reinforcement_learning_planner::reward &msg)
{
    m_reward_msg = msg;
}

void OfflineCollect::action_callback(const reinforcement_learning_planner::action &msg)
{
    m_action_sub_msg = msg;
}

void OfflineCollect::get_state_reward()
{
    semantic_line_state next_state = {m_state_msg.offset, m_state_msg.special_case};

    double reward = 0;

    if (m_reward_msg.offset == 1 || m_reward_msg.offset == -1 || m_reward_msg.offset == 0)
    {
        reward = 0.5;
    }
    else if (m_reward_msg.offset == 2 || m_reward_msg.offset == -2)
    {
        reward = 0.3;
    }
    else if (m_reward_msg.offset == 3 || m_reward_msg.offset == -3)
    {
        reward = 0.1;
    }
    if (m_reward_msg.out_of_line)
    {
        reward = -10.0;
    }

    m_rl_handler.set_next_state(reward, next_state);
}

driving_action OfflineCollect::get_remote_action()
{
    driving_action new_action = {m_action_sub_msg.angular_action, m_action_sub_msg.linear_action};
    return new_action;
}

void OfflineCollect::set_action(const driving_action &new_action)
{
    m_rl_handler.set_action(new_action);
    m_action_pub_msg.linear_action = new_action.linear_discretization;
    m_action_pub_msg.angular_action = new_action.angular_discretization;
    m_action_pub_msg.revert = false;

    m_pub_action.publish(m_action_pub_msg);
    ROS_INFO("Setting action:%d, %d", m_action_pub_msg.angular_action, m_action_pub_msg.linear_action);
}