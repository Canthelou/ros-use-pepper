#include "VisualPathFollowingWithKinect.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_path_following_with_kinect");
    VisualPathFollowingWithKinect prog;

    ros::spin();
    return 0;
}
