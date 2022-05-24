#include "cam/cam.h"
#include "ltproc/ltproc.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "offset_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub;
    Cam cam(nh);
    ltproc ltproc(nh);
    ros::Rate loop_rate(30);
    sub = it.subscribe("/camera/infra/image_raw", 10, &Cam::Callback, &cam);
    while(ros::ok())
    {
         ltproc.Capture(cam.frame);
         ltproc.set_window(640,480);
         ltproc.lt_proc();
    //     ltproc.show_result(240,180);
        if(cv::waitKey(5) == 27)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

