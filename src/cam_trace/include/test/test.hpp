#ifndef _TEST_H_
#define _TEST_H_

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>

class Test
{
public:
    Test();
    ~Test();

    void print(std::string str);

private:
    int a;
    int b;
    int c;
};

#endif
