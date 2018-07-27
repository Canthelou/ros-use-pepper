#include <ros/ros.h>
#include "TeleopRobot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_robot");
    TeleopRobot teleop_robot;

    ros::spin();
}
