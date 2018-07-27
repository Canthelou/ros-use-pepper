#include "VisualServoTools.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include "visp_bridge/image.h"
#include <visp/vpDisplayX.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>


#define TAKE_REGLAR_FRAMES 0
#define TAKE_FRAMES_USING_VISUAL_SERVO 1


using namespace std;

class PathManager
{

public:

    PathManager();
    ~PathManager();

    void initPathManager();
    void initValues();
    void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomsg);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void callbackTimer(const ros::TimerEvent&);
    void robotVelocityCallback(const geometry_msgs::TwistConstPtr& twist_vs);
    void write_data();
    void reverse_vectors();
    double get_position_in_x(const nav_msgs::Odometry& odomsg);


private:


    bool m_pure_rotation;
    bool m_vs_translation;
    int m_iter;
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    ros::Publisher m_ros_pub_im_filename;
    ros::Subscriber m_robot_sub;
    ros::Subscriber m_joystick_sub;
    ros::Subscriber m_odom_sub;

    int m_linear, m_angular;
    double m_l_scale, m_a_scale;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_desired_image_rotated;
    vpImage<unsigned char> m_prev_desired_image;
    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_mask_image;
    vpImage<unsigned char> m_init_image;


    ofstream m_logfile;
    ofstream m_logerror;
    ofstream m_log_poly;
    ofstream m_log_vel;
    ofstream m_log_odom;

    string m_logs_path;
    string m_data_path;

    CFeatureLuminanceOmni *m_sId ;
    //    CFeatureLuminanceOmni m_sI ;
    std::set<int> m_set_indexes;
    vpMatrix m_Lsd;
    vpMatrix m_Hsd;
    vpMatrix m_diagHsd;

    std::vector<vpImage<unsigned char> > m_vec_desired_images;
    std::vector<vpMatrix> m_vec_Lsd;
    std::vector<vpMatrix> m_vec_Hsd;
    std::vector<vpMatrix> m_vec_diagHsd;
    std::vector<CFeatureLuminanceOmni *> m_vec_sId;

    vector<CFeatureLuminanceOmni*>::iterator m_it_vec_sId;
    vector<vpMatrix>::iterator m_it_vec_Lsd;
    vector<vpMatrix>::iterator m_it_vec_Hsd;
    vector<vpMatrix>::iterator m_it_diagHsd;


    vpColVector m_error ;

    int m_width;
    int m_height;

    //    int m_iter;
    geometry_msgs::Twist m_velocity;
    geometry_msgs::Twist m_listened_velocity;


    //    std::ofstream m_logfile;
    //    vpImage<unsigned char> my_image;
    //    int m_index_desired_image_fwd;
    int m_index_desired_image_fwd;
    int m_index_desired_image_bwd;
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_previous_desired_image;
    vpDisplayX m_display_error;
    //    double m_current_error;
    //    double m_pred_error;
    //    double m_mu;
    //    double m_lambda;
    bool m_save_desired_features;
    bool m_move_forward;
    bool m_move_backward;
    bool m_demi_tour;
    bool m_move_backward_visual_compass;

    double m_rho;
    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_center_rotation[2];

    bool m_flag;

    CFeatureLuminanceOmni m_sI ;
    int m_nb_decroissace_error;
    bool m_acquisition;
    int m_nb_backward;

    int m_nb_velocity_null;

    double m_ratio_threshold;
    double m_diff_threshold;
    int m_max_iters;
    ros::Timer m_timer;

    //    VisualServoTools m_vs_backward;
    VisualServoTools m_vs_forward;
    VisualServoTools m_vs_backward;

    bool m_visual_servo_mode;
    bool m_pure_rotation_mode;

    double m_first_it_vel_x;
    double m_prev_vel_x;
    double m_curr_vel_x;

    double m_first_it_vel_y;
    double m_prev_vel_y;
    double m_curr_vel_y;

    double m_threshold_error;
    double m_center_image[2];
    double m_required_angle;
    bool m_translation_mode;
    bool m_rotation_mode;
    bool m_rotation_done;
    bool m_move_backward_without_pure_rotation;
    bool m_move_forward_without_pure_rotation;

    int m_index_courant;

    nav_msgs::Odometry m_current_pose;
    double m_prev_position;
    double m_current_position;
};
