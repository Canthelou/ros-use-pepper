#include <ros/ros.h>
#include "Sonar.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar");
    Sonar sonar;

    ros::spin();
}
