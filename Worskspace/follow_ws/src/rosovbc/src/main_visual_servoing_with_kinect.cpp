#include "VisualServoingWithKinect.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_servoing_with_kinect");
    VisualServoingWithKinect prog;

    ros::spin();
    return 0;
}
