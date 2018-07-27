#include "VCompass.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vs_with_visual_compass");
    VCompass prog;

    ros::spin();
    return 0;
}



