#include "Odometry.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry");
    Odometry prog;

    ros::spin();
    return 0;
}
