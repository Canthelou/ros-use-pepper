#include "VisualPathFollowingWithRotation.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_path_following_with_rotation");
    VisualPathFollowingWithRotation prog;

    ros::spin();
    return 0;
}
