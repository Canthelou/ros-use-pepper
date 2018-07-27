#include "VisualPathFollowingWithPureRotation.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_path_following_with_pure_rotation");
    VisualPathFollowingWithPureRotation prog;

    ros::spin();
    return 0;
}
