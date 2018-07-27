#include "PathManagerEnhanced.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_manager_enhanced");
    PathManagerEnhanced path_manager_enhanced;
    ros::spin();
    return 0;
}
