#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"
#include <fstream>

using namespace std;

class Master
{

public:

    Master()
    {
        string sonarMovements, visualServoMovements, joystickMovements, robotMovements;
        m_nh.param("sonarMovements", sonarMovements, string(""));
        m_nh.param("visualServoMovements", visualServoMovements, string(""));
        m_nh.param("joystickMovements", joystickMovements, string(""));


        m_nh.param("robotMovements", robotMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));

        m_nh.getParam("sonarMovements", sonarMovements);
        m_nh.getParam("visualServoMovements", visualServoMovements);
        m_nh.getParam("joystickMovements", joystickMovements);

        m_nh.getParam("logs", m_logs_path);

        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        //
        // subscription to sonarMovemets topic
        //
        m_sonar_sub = m_nh.subscribe<geometry_msgs::Twist>(sonarMovements, 1, &Master::sonarCallback, this);

        //
        // subscription to visualServoMovements topic
        //
        m_visualServo_sub = m_nh.subscribe<geometry_msgs::Twist>(visualServoMovements, 1, &Master::visualServoCallback, this);


        //
        // subscription to joystickMovements topic
        //
        m_joystick_sub = m_nh.subscribe<geometry_msgs::Twist>(joystickMovements, 1, &Master::joystickCallback, this);


        //
        // advertisement to robotTopic
        //
        m_ros_pub  = m_nh.advertise<geometry_msgs::Twist>(robotMovements, 1000);

    }

    void sonarCallback(const geometry_msgs::TwistConstPtr& twist_sonar)
    {
//        m_velocity.linear = twist_sonar->linear;
//        m_velocity.angular = twist_sonar->angular;
//        m_logfile<<"in sonarCallback"<<m_velocity<<endl;

//        m_ros_pub.publish(m_velocity);
//        ros::spinOnce();
    }

    void rosariaCallback ()
    {

    }

    void joystickCallback(const geometry_msgs::TwistConstPtr& twist_joy)
    {

        m_velocity.linear = twist_joy->linear;
        m_velocity.angular = twist_joy->angular;
        m_logfile<<"in joystickCallback"<<m_velocity<<endl;
        m_ros_pub.publish(m_velocity);
        ros::spinOnce();

    }

    void visualServoCallback(const geometry_msgs::TwistConstPtr& twist_vs)
    {

//        m_velocity.linear = twist_vs->linear;
//        m_velocity.angular = twist_vs->angular;
//        m_logfile<<"in visualServoCallback"<<m_velocity<<endl;

//        m_ros_pub.publish(m_velocity);
//        ros::spinOnce();

    }

    ~Master()
    {

    }

private:
    ros::NodeHandle m_nh;

    ros::Publisher m_ros_pub;
    ros::Subscriber m_sonar_sub;
    ros::Subscriber m_visualServo_sub;
    ros::Subscriber m_joystick_sub;


    ofstream m_logfile;
    string m_logs_path;

    geometry_msgs::Twist m_velocity;


};
