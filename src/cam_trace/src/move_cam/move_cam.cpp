#include "move_cam/move_cam.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"move_cam");
    ros::NodeHandle nh;
    Move move(nh);
    move.setposition();
   // move.run();
    return 0;
}
