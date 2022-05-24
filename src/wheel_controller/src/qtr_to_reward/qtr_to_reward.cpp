#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <reinforcement_learning_planner/reward.h>
#include <iostream>

class reward_transform
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;

        reinforcement_learning_planner::reward r_msg;
    public:
        reward_transform(const ros::NodeHandle &nh);
        void callback(const std_msgs::UInt16MultiArray &msg);
        void start();
        void translate(int x[8]);
        void execute();
};

reward_transform::reward_transform(const ros::NodeHandle &nh)
    :nh(nh)
{
    ROS_INFO("class reward_transform has been constructed");
}

void reward_transform::start()
{
    pub = nh.advertise<reinforcement_learning_planner::reward>("/reward",1);
    sub = nh.subscribe("/pub_qtr",1,&reward_transform::callback, this);
}

void reward_transform::callback(const std_msgs::UInt16MultiArray &msg)
{
    int reward[8];
    for(uint8_t i=0; i<8; i++)
    {
        reward[i] = msg.data[i];
    }
    translate(reward);
}

void reward_transform::translate(int x[8])
{
    int count = 0;//沒有在線上
    for(int i=0;i<8;i++)
    {
        if(x[i]<600)
        {
            count++;
        }
    }
    if(count==8)
    {
        r_msg.out_of_line = 1;
    }

    else if(count<8)
    {
        //判斷是否超出線

        if(x[0] >1500 || x[7] > 1500)//碰到最旁邊輪子
        {
            r_msg.out_of_line = 1;
        }
        else
        {
            r_msg.out_of_line = 0;
            //判斷偏左偏右

            int value = x[0]*0.8+x[1]*0.6+x[2]*0.4+x[3]*0.2+x[4]*-0.2+x[5]*-0.4+x[6]*-0.6+x[7]*-0.8;

            if(value < 600 && value > -600)
            {
                r_msg.offset = 0;
            }

            //需要左轉
            if(value>=600 && value <1200)
            {
                r_msg.offset = -1;
            }
            if(value>=1200 && value <1800)
            {
                r_msg.offset = -2;
            }
            if(value>=1800)
            {
                r_msg.offset = -3;
            }

            //需要右轉
            if(value<= -600 && value > -1200)
            {
                r_msg.offset = 1;
            }
            if(value<= -1200 && value > -1800)
            {
                r_msg.offset = 2;
            }
            if(value<= -1800)
            {
                r_msg.offset = 3;
            }
            //std::cout << value << std::endl;
        }
    }
}

void reward_transform::execute()
{
    ros::Rate loop_rate(100);
    ros::spinOnce();
    pub.publish(r_msg);
    loop_rate.sleep();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "qtr_to_reward");
    ros::NodeHandle nh;

    reward_transform reward_transform(nh);
    reward_transform.start();
    while(ros::ok())
    {
        reward_transform.execute();
    }
}


