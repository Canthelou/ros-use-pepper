#include <ros/ros.h>
//#include <turtlesim/Velocity.h>
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"

#include <stdlib.h>
#include <fstream>
using namespace std;

class TeleopRobot
{
public:
    TeleopRobot();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);


private:

    ros::NodeHandle m_nh;
    int m_linear, m_angular;
    double m_l_scale, m_a_scale;
    ros::Publisher m_vel_pub;
    ros::Subscriber m_joy_sub;
    string m_logs_path, m_joy_movements_topic_name;
    std::ofstream m_logfile;

};


