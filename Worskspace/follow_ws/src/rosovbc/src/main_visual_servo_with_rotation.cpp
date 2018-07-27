#include "VisualServoWithRotation.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_servoing_with_rotation");
    VisualServoWithRotation prog;

    ros::spin();
    return 0;
}
