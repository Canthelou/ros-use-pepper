#include "VCompassDecoupled.h"
#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vs_decoupled");
    VCompass prog;

    ros::spin();
    return 0;
}
