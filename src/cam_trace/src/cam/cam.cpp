#include "cam/cam.h"
#include "ltproc/ltproc.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offset_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub;
    Cam cam(nh);
    ltproc ltproc(nh);

    bool show_result = false;
    int cam_rate = 10;

    nh.getParam("show_result", show_result);
    nh.getParam("cam_rate", cam_rate);

    ros::Rate loop_rate(cam_rate);
    //   sub = it.subscribe("/camera/infra/image_raw", 10, &Cam::Callback, &cam);
    while (ros::ok())
    {
        cam.Capture();
        ltproc.Capture(cam.frame);
        ltproc.set_window(640, 480);
        ltproc.lt_proc();
        if (show_result)
            ltproc.show_result(240, 180);
        if (cv::waitKey(5) == 27)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
