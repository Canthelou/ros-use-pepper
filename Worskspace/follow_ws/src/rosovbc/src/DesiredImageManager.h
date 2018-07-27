#include "VisualServoTools.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include "visp_bridge/image.h"



using namespace std;

class DesiredImageManager
{

public:

    DesiredImageManager();

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    ~DesiredImageManager();

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;

    double m_center_image[2];
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_mask_image;

    int m_width;
    int m_height;

    int m_iter;
    geometry_msgs::Twist m_velocity;

    std::ofstream m_logfile;
    string m_logs_path;
    string m_data_path;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    VisualServoTools m_visual_servo_tools;


};
