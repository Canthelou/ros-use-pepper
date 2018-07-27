#include "PhotoVisualServo.h"

#include <ros/ros.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "photometric_visual_servo");
    PhotoVisualServo prog;

    ros::spin();
    return 0;
}
