#include "PathManager.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_manager");
    PathManager path_manager;
    ros::spin();
    return 0;
}
