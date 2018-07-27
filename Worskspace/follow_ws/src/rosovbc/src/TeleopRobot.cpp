#include "TeleopRobot.h"

TeleopRobot::TeleopRobot():
    m_linear(0),
    m_angular(0)
{


    m_nh.param("axis_linear", m_linear, m_linear);
    m_nh.param("axis_angular", m_angular, m_angular);
    m_nh.param("scale_angular", m_a_scale, m_a_scale);
    m_nh.param("scale_linear", m_l_scale, m_l_scale);
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("joystickMovements", m_joy_movements_topic_name, string(""));

    m_nh.getParam("joystickMovements", m_joy_movements_topic_name);
    m_nh.getParam("logs", m_logs_path);


    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

//    string robotTopic = "/RosAria/cmd_vel";
    string joystickTopic =  "joy";



    m_vel_pub = m_nh.advertise<geometry_msgs::Twist>(m_joy_movements_topic_name, 10);
    m_joy_sub = m_nh.subscribe<sensor_msgs::Joy>(joystickTopic, 10, &TeleopRobot::joyCallback, this);

}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = m_a_scale*joy->axes[m_angular];
    vel.linear.x = m_l_scale*joy->axes[m_linear];
//    ROS_INFO("velocity %f %f\n", vel.angular.z, vel.linear.x);
//    m_logfile<<"linear "<<vel.linear.x<<" "<<vel.angular.z<<endl;
    m_vel_pub.publish(vel);
}
