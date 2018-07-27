#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"
#include <fstream>

using namespace std;

class Odometry
{

public:

    Odometry()
    {

        string sonarTopic;

        m_nh.param("sonarTopic", sonarTopic, string(""));

        m_nh.param("logs", m_logs_path, string(""));

        m_nh.getParam("sonarTopic", sonarTopic);

        m_nh.getParam("logs", m_logs_path);



        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());


        //
        // subscription to sonar topic
        //
        m_sonar_sub = m_nh.subscribe<sensor_msgs::PointCloud>(sonarTopic, 1, &Odometry::sonarCallback, this);

        //
        //
        // subscription

    }

    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud)
    {

    }

    void rosariaCallback ()
    {

    }

    void visualServoingCallback()
    {

    }

    ~Odometry()
    {

    }

private:
    ros::NodeHandle m_nh;

    ros::Publisher m_ros_pub;
    ros::Subscriber m_sonar_sub;

    ofstream m_logfile;
    string m_logs_path;

    geometry_msgs::Twist m_velocity;


};
