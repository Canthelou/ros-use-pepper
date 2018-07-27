#include "DesiredImageManager.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "desired_image_manager");
    DesiredImageManager prog;
    ros::spin();
    return 0;
}
