#include "offline_collect/offline_collect.hpp"

OfflineCollect::OfflineCollect()
{
    ROS_INFO("Default OfflineCollect constructed");
}

OfflineCollect::OfflineCollect(ros::NodeHandle &nh)
    : m_nh(nh),
      m_sub_state(nh.subscribe("/state", 1, &OfflineCollect::state_callback, this)),
      m_sub_reward(nh.subscribe("/reward", 1, &OfflineCollect::reward_callback, this)),
      m_pub_action(nh.advertise<reinforcement_learning_planner::action>("/action", 1)),
      m_planner_state(PlannerState::INIT),
      m_exit(false)
{
    ROS_INFO("OfflineCollect constructed");
}

OfflineCollect::~OfflineCollect()
{
    ROS_INFO("OfflineCollect destructed");
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
    //initialize first state
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

void OfflineCollect::plan()
{
    int execute_rate = 30;
    m_nh.getParam("execute_rate", execute_rate);
    ros::Rate time_step(execute_rate);
    ROS_INFO("plan_q_learning");

    m_rl_handler.get_action_epsilon();

    set_action();
    time_step.sleep(); //giving some time to react and observe the state/reward
    ros::spinOnce();
    get_state_reward();

    m_rl_handler.learn();
    m_rl_handler.update_state();
}

void OfflineCollect::state_callback(const reinforcement_learning_planner::state::ConstPtr &msg)
{
    ROS_INFO("OfflineCollect state_callback");
}

void OfflineCollect::reward_callback(const reinforcement_learning_planner::reward::ConstPtr &msg)
{
    ROS_INFO("OfflineCollect reward_callback");
}

void OfflineCollect::get_state_reward()
{
    semantic_line_state next_state = {m_state_msg.offset};
    m_rl_handler.set_next_state(m_reward_msg.offset, next_state);
}

void OfflineCollect::set_action()
{
    ROS_INFO("OfflineCollect set_action");
}