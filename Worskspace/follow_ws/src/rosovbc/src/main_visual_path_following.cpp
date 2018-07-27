#include "VisualPathFollowing.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_path_following");
    VisualPathFollowing prog;

    ros::spin();
    return 0;
}
