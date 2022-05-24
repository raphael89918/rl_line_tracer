#include "demo/demo.hpp"

Demo::Demo()
{
    ROS_INFO("Default Demo constructed");
}

Demo::Demo(ros::NodeHandle &nh, DemoChoice demo_choice)
    : m_nh(nh),
      m_sub_state(nh.subscribe("/state", 1, &Demo::state_callback, this)),
      m_pub_action(nh.advertise<reinforcement_learning_planner::action>("/wheel/control", 1)),
      m_demo_choice(demo_choice),
      m_rl_handler(choice_to_filepath()),
      m_planner_state(PlannerState::INIT),
      m_exit(false)
{
    ROS_INFO("OfflineCollect constructed");
}

Demo::~Demo()
{
    ROS_INFO("OfflineCollect destructed");
}

void Demo::init()
{
    m_rl_handler.init();
    m_rl_handler.load_model(m_rl_handler.get_recent_filename());
}

void Demo::start()
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

void Demo::suspend()
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

void Demo::execute()
{
    //initialize first state
    ros::spinOnce();
    semantic_line_state state_trait = {m_state_msg.offset};
    //m_rl_handler.set_state(m_reward_msg.offset, state_trait);

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

void Demo::stop_wheel()
{
    m_action_pub_msg.linear_action = 0;
    m_action_pub_msg.angular_action = 0;
    m_pub_action.publish(m_action_pub_msg);
}

void Demo::plan()
{
    int execute_rate = 30;
    m_nh.getParam("execute_rate", execute_rate);
    ros::Rate time_step(execute_rate);

    m_rl_handler.best_action();

    set_action();
    time_step.sleep(); //giving some time to react and observe the state/reward
    ros::spinOnce();
    get_state();

    m_rl_handler.update_episode();
    m_rl_handler.update_state();
}

void Demo::state_callback(const reinforcement_learning_planner::state &msg)
{
    m_state_msg = msg;
}

void Demo::get_state()
{
    semantic_line_state next_state = {m_state_msg.offset};
    m_rl_handler.set_next_state(0, next_state);
}

void Demo::set_action()
{
    m_action_pub_msg.linear_action = m_rl_handler.action.trait().linear_discretization;
    m_action_pub_msg.angular_action = m_rl_handler.action.trait().angular_discretization;
    m_action_pub_msg.revert = false;

    m_pub_action.publish(m_action_pub_msg);
    ROS_INFO("Setting action:%d, %d", m_action_pub_msg.angular_action, m_action_pub_msg.linear_action);
}

std::string Demo::choice_to_filepath()
{
    if (m_demo_choice == DemoChoice::OFFLINE)
    {
        return "/home/ical/rl_line_tracer/rl_model/offline";
    }
    if (m_demo_choice == DemoChoice::ONLINE)
    {
        return "/home/ical/rl_line_tracer/rl_model/online";
    }
}
