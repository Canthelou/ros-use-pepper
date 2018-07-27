#include "VisualServoTools.h"


#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include "visp_bridge/image.h"
#include <std_msgs/String.h>


using namespace std;

class VisualPathFollowingWithPureRotation
{

public:

    VisualPathFollowingWithPureRotation();
    ~VisualPathFollowingWithPureRotation();
    void reverse_vectors();
    void initVisualPathFollowingWithPureRotation();
    void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    ros::Publisher m_ros_pub_im_file_name;
    ros::Subscriber m_sonar_sub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_prev_desired_image;

    vpImage<unsigned char> m_desired_image_rotated;


    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_difference_image_translation;

    vpImage<unsigned char> m_mask_image;

    ofstream m_logfile;
    ofstream m_logerror;
    ofstream m_logvisualcompass;


    std::vector<string> m_vec_str_Ls;
    std::vector<string> m_vec_str_Hs;
    std::vector<string> m_vec_str_diagHs;

    string m_logs_path;
    string m_data_path;

    std::vector<vpImage<unsigned char> > m_vec_desired_images;
    std::vector<vpImage<unsigned char> > m_current_images;
    std::vector<vpImage<unsigned char> > m_diff_images;


    int m_index_desired_image_fwd;
    vector<CFeatureLuminanceOmni*> m_vec_sId;
    vector<vpMatrix > m_vec_Lsd;
    vector<vpMatrix > m_vec_Hsd;
    vector<vpMatrix > m_vec_diagHsd;

    vector<CFeatureLuminanceOmni*>::iterator m_it_vec_sId;
    vector<vpMatrix>::iterator m_it_vec_Lsd;
    vector<vpMatrix>::iterator m_it_vec_Hsd;
    vector<vpMatrix>::iterator m_it_diagHsd;


    vector<CFeatureLuminanceOmni*>::iterator m_it_vec_sId_backward;
    vector<vpMatrix>::iterator m_it_vec_Lsd_backward;
    vector<vpMatrix>::iterator m_it_vec_Hsd_backward;
    vector<vpMatrix>::iterator m_it_diagHsd_backward;


    CFeatureLuminanceOmni m_sI ;
    int m_nb_croissance;

    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    geometry_msgs::Twist m_velocity;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    vpDisplayX m_display_rotated_desired_image;
    vpDisplayX m_display_prev;


    double m_mu;
    double m_lambda;
    double m_alpha;
    int m_first_forward;
    int m_first_backward;
    int m_nb_tours;
    double m_threshold_error;
    double m_diff_error;
    double m_mean_diff_error;
    double m_rho;
    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;

    double m_center[2];
    double m_center_image[2];
    double m_theta_degrees;
    bool m_move_forward;
    bool m_move_backward;

    VisualServoTools m_combined_rotation_and_translation;
    CFeatureLuminanceOmni m_sI_combined_translation_and_rotation;
    CFeatureLuminanceOmni *m_sId_combined_trans_and_rot;
    vpMatrix m_Lsd, m_Hsd, m_diagHsd;
    vpColVector m_prev_vel;
    vpColVector m_curr_vel;

    VisualServoTools m_translation;

    CFeatureLuminanceOmni *m_sId_translation;
    CFeatureLuminanceOmni m_sI_translation;

    bool m_translation_mode;
    bool m_rotation_mode;
    bool m_flag;
    VisualServoTools m_vs_forward;
};
