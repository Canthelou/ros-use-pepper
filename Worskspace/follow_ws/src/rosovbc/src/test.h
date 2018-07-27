#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visp/vpImage.h>
#include <visp_bridge/image.h>
#include "VisualServoTools.h"
#include <fstream>
#include <tf/transform_broadcaster.h>

using namespace std;

class Test
{

public:

    Test()
        : m_it(m_nh)
    {
        m_nh.getParam("logs", m_logs_path);
        string robotTopic, cameraTopic;
        m_nh.getParam("robotTopic", robotTopic);
        m_nh.getParam("cameraTopic", cameraTopic);

        stringstream ss_logs;
        ss_logs<<m_logs_path<<"logfile_test_odom.txt";
        m_logfile.open(ss_logs.str().c_str());
        m_logfile<<"opening logfile for odometry information"<<endl;

        m_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("RosAria/pose", 1, &Test::robotOdometryCallback, this);
        m_image_sub = m_it.subscribe(cameraTopic, 1, &Test::imageCallback, this);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1000);
    }

    void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
    {
        m_logfile<<"################################ call to robot odometry callback"<<endl;
        m_logfile<<odomsg->pose.pose.position.x<<"\t"<<odomsg->pose.pose.position.y<<"\t"<<odomsg->pose.pose.position.z<<endl;

    }


    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {
        m_logfile<<"################################ call to imageCallback"<<endl;
        m_velocity.linear.x = 0.1;
        m_ros_pub.publish(m_velocity);
        ros::spinOnce();
    }


    ~Test()
    {

    }


private:

    std::vector<vpImage<unsigned char> > m_initial_images;
    vpImage<unsigned char> m_desired_image;
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    ros::Subscriber m_sonar_sub;
    ros::Subscriber m_odom_sub;

    ofstream m_logfile;
    ofstream m_ofs_angles;
    string m_logs_path;
    string m_data_path;
    vpImage<unsigned char> m_mask;//(m_vs_tools.getHeight(), m_vs_tools.getWidth());
    vpImage<unsigned char> m_initial_image;
    vpDisplayX m_display_init;
    vpDisplayX m_display_desired;
    int m_iter;
    geometry_msgs::Twist m_velocity;

    VisualServoTools m_vs_tools;


};


