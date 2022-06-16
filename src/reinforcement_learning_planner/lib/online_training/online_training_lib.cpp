#include "online_training/online_training.hpp"
#include <ctime>

// using rl_state = relearn::state<semantic_line_state>;
// using rl_action = relearn::action<driving_action>;
// using rl_episode = relearn::link<rl_state, rl_action>;

OnlineTraining::OnlineTraining()
    : m_execute_rate(get_execute_rate())
{
    ROS_INFO("Default class OnlineTraining has been constructed");
}

OnlineTraining::OnlineTraining(ros::NodeHandle &nh)
    : m_nh(nh),
      m_sub_state(nh.subscribe("/state", 1, &OnlineTraining::state_callback, this)),
      m_sub_reward(nh.subscribe("/reward", 1, &OnlineTraining::reward_callback, this)),
      m_pub_action(nh.advertise<reinforcement_learning_planner::action>("/wheel/control", 1)),
      m_execute_rate(get_execute_rate()),
      m_rl_handler("/home/ical/rl_line_tracer/rl_model/online"),
      m_planner_state(PlannerState::INIT),
      m_exit(false)
{
    ROS_INFO("Class OnlineTraining has been constructed");
}

OnlineTraining::~OnlineTraining()
{
    ROS_INFO("Class OnlineTraining has been destroyed");
}

int OnlineTraining::get_execute_rate()
{
    int execute_rate;
    m_nh.getParam("execute_rate", execute_rate);
    return execute_rate;
}

void OnlineTraining::init()
{
    m_rl_handler.init();
    m_rl_handler.load_model(m_rl_handler.get_recent_filename());
}

void OnlineTraining::start()
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

void OnlineTraining::suspend()
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
                stop_wheel();
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

void OnlineTraining::execute()
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
                stop_wheel();
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

void OnlineTraining::stop_wheel()
{
    m_action_msg.linear_action = 0;
    m_action_msg.angular_action = 0;
    m_pub_action.publish(m_action_msg);
}

void OnlineTraining::rotate_wheel()
{
    m_action_msg.linear_action = 0;
    m_action_msg.angular_action = 1;
    m_pub_action.publish(m_action_msg);
}

void OnlineTraining::revert_wheel()
{
    if (m_rl_handler.is_revert_vector_empty())
    {
        ROS_INFO("cannot revert, now rotate wheel");
        rotate_wheel();
        return;
    }
    rl_episode episode = m_rl_handler.pop_revert_vector();

    m_action_msg.linear_action = -episode.action.trait().linear_discretization;
    m_action_msg.angular_action = -episode.action.trait().angular_discretization;
    m_pub_action.publish(m_action_msg);

    ROS_INFO("Revert action: %d, %d", m_action_msg.angular_action, m_action_msg.linear_action);

    return;
}

void OnlineTraining::plan()
{
    auto action = m_rl_handler.get_action_epsilon();
    set_action(action);

    m_rl_handler.push_revert_vector();
    m_execute_rate.sleep(); // giving some time to react and observe the state/reward

    ros::spinOnce();
    get_state_reward();

    out_of_bounds_trap();

    m_rl_handler.learn();
    m_rl_handler.record_episode();
    m_rl_handler.update_state();
}

void OnlineTraining::state_callback(const reinforcement_learning_planner::state &msg)
{
    m_state_msg = msg;
}

void OnlineTraining::reward_callback(const reinforcement_learning_planner::reward &msg)
{
    m_reward_msg = msg;
}

void OnlineTraining::get_state_reward()
{
    semantic_line_state next_state = {m_state_msg.offset};

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
        reward = -0.1;
    }
    if (m_reward_msg.out_of_line)
    {
        reward = -10.0;
    }

    m_rl_handler.set_next_state(reward, next_state);
}

void OnlineTraining::set_action(const driving_action &new_action)
{
    m_action_msg.linear_action = new_action.linear_discretization;
    m_action_msg.angular_action = new_action.angular_discretization;
    m_action_msg.revert = false;

    m_pub_action.publish(m_action_msg);
    ROS_INFO("Setting action: %d, %d", m_action_msg.angular_action, m_action_msg.linear_action);
}

void OnlineTraining::out_of_bounds_trap()
{
    while (m_reward_msg.out_of_line == 1 || m_state_msg.special_case == 1)
    {
        revert_wheel();
        m_execute_rate.sleep();
        ros::spinOnce();
    }
}
