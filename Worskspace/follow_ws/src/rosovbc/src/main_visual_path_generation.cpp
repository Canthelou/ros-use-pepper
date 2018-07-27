#include "visual_path_generation.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_path_generation");

    VisualPathGeneration prog;

    ros::spin();
    return 0;
}

