#include "VisualServoing.h"
#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_servoing");
    VisualServoing prog;

    ros::spin();
    return 0;
}
