#include "VisualServoTools.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//openCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include "visp_bridge/image.h"
#include <std_msgs/String.h>

//Libraries de projection
#include "camera/CModelStereo.h"
#include "camera/CModelStereoXml.h"
#include <camera/CPoint.h>
#include <camera/CPerspective.h>

#include <camera/CModel.h>

using namespace std;
using namespace cv;

class VisualServoingWithKinect
{

public:

    VisualServoingWithKinect();
    ~VisualServoingWithKinect();

    void imageCallbackuEye(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackrgb(const sensor_msgs::ImageConstPtr& msg);
    void initValues();
    void initVisualServoing();
    void buildDesiredFeaturesWithDepth(vpImage<unsigned char>& desired_image, vpImage<float>& depth,
                                                  const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                                                  const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, std::ostream& logfile = std::cout);

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_sub_omni_image;
    image_transport::Subscriber m_sub_kinect_depth;
    image_transport::Subscriber m_sub_kinect_rgb;


    ros::Publisher m_ros_pub;
    ros::Publisher m_ros_pub_im_file_name;
    ros::Subscriber m_sonar_sub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_desired_image_rotated;
    vpImage<unsigned char> m_mask_image;
    vpImage<float> m_depth_image;
    vpImage<float> m_desired_depth_image;

    ofstream m_logfile;
    ofstream m_logerror;

    CCameraOmniParameters m_cam_param;

    std::vector<string> m_vec_str_Ls;
    std::vector<string> m_vec_str_Hs;
    std::vector<string> m_vec_str_diagHs;

    string m_logs_path;
    string m_data_path;


    CFeatureLuminanceOmni m_sI ;
    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    geometry_msgs::Twist m_velocity;

    cv_bridge::CvImagePtr m_cv_ptr_omni;
    cv_bridge::CvImagePtr m_cv_ptr_kinect_rgb;

    VisualServoTools m_vs_tools;

    CFeatureLuminanceOmni *m_desired_features;
    CFeatureLuminanceOmni m_current_features;
    vpImage<unsigned char> m_difference_image;
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;

    double m_rho, m_px, m_py, m_u0, m_v0, m_xi, m_center[2], m_threshold_error, m_lambda, m_mu, m_current_error;
    vpMatrix m_Lsd, m_Hsd, m_diagHsd;

};


