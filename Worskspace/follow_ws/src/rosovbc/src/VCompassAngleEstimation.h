#include "VisualServoTools.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <image_transport/image_transport.h>
#include "visp_bridge/image.h"
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

//static const char WINDOW[] = "Image window";

using namespace std;

class VCompass
{

public:

    VCompass()
        : m_it(m_nh)
    {
        ROS_DEBUG_NAMED("test_only", "Hello %s", "World");

        m_current_distance                  = 0;
        m_iter_g                            = 0;
        m_iter                              = 0;
        m_nb_cycles                         = 0;
        m_nb_tours                          = 0;
        m_mean_diff_error                   = 0.0;
        m_diff_error                        = 0.0;
        m_first_forward                     = 0;
        m_first_backward                    = 0;
        m_width                             = 656;
        m_height                            = 656;

        m_first_rotation_mode               = true;
        m_second_rotation_mode              = false;
        m_translation_mode                  = false;

        m_required_angle                    = 0.0;
        m_required_angle_degrees            = 0.0;

        m_nb_croissance                     = 0;
        m_index_desired_image_fwd           = 1;
        m_index_desired_image_bwd           = -1;

        m_px                                = m_visual_servo_tools.getPx();
        m_py                                = m_visual_servo_tools.getPy();
        m_u0                                = m_visual_servo_tools.getU0();
        m_v0                                = m_visual_servo_tools.getV0();
        m_xi                                = m_visual_servo_tools.getXi();
        m_center[0]                         = m_u0;
        m_center[1]                         = m_v0;


        string cameraTopic, robotTopic, visualServoMovements, poseTopic;
        m_nh.param("cameraTopic", cameraTopic, string(""));
        m_nh.param("robotTopic", robotTopic, string(""));
        m_nh.param("poseTopic", poseTopic, string(""));
        m_nh.param("visualServoMovements", visualServoMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path,string(""));
        m_nh.param("error_threshold",m_threshold_error, double(0.0));
        //        m_nh.param("lambda", m_lambda, double(10.0));

        m_nh.getParam("poseTopic", poseTopic);
        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("visualServoMovements", visualServoMovements);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        m_nh.getParam("error_threshold",m_threshold_error);
        m_nh.getParam("robotTopic", robotTopic);
        //        m_nh.getParam("lambda", m_lambda);

        this->initValues();
        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        stringstream str_error;
        str_error<<m_logs_path<<"error.mat";
        m_logerror.open(str_error.str().c_str());

        stringstream str_angles;
        str_angles<<m_logs_path<<"angles.mat";
        m_log_angles.open(str_angles.str().c_str());

        m_current_image.init(m_height, m_width);
        m_desired_image.init(m_height, m_width);
        m_difference_image.init(m_height, m_width);
        m_desired_image_rotated.init(m_height, m_width);

        m_new_desired_image_rotated.init(m_height, m_width);

        m_difference_image_translation.init(m_height, m_width);
        m_difference_image_rotation.init(m_height, m_width);
        m_temp_desired_image.init(m_height, m_width);

        m_display_current_image.init(m_current_image, 0, 0, "Current image");

        m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
//        m_display_desired_image_rot.init(m_desired_image_rotated, m_width+10, m_height, "Desired image rotated");
//        m_display_desired_image_rot.init(m_new_desired_image_rotated, m_width+10, m_height, "New Desired Image Oriented");

        m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");
//        m_display_error_translation.init(m_difference_image_translation, 0, m_height, "Difference translation");
//        m_display_error_rotation.init(m_difference_image_rotation, 2*m_width+10, m_height, "Difference rotation 2");
        m_mask_image.init(m_height, m_width);

        m_image_sub = m_it.subscribe(cameraTopic, 1, &VCompass::imageCallback, this);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1);
        m_lambda_rotation = 5.0;
        initVCompass();
    }

    ~VCompass()
    {
        delete m_sId_translation;
        delete m_sId_first_rotation;
        delete m_sId_second_rotation;
        m_logfile.close();
    }

    void initValues()
    {
        this->m_current_error = 9999;
        m_iter      = 0;
        m_mu        = 0.01;
        //m_lambda  = 50 ;
        m_nh.getParam("lambda", m_lambda);

        m_nb_croissance = 0;
        m_mean_diff_error = 0.0;
        m_diff_error = 0.0;
    }

    void initVCompass()
    {
        m_sad = 999.0;
        std::string filename_read_image, filename_image_mask;

        // read desired image
        stringstream sstream_desired;
        sstream_desired<<m_logs_path<<"desired_images/desired_image.png";
        filename_read_image = vpIoTools::path(sstream_desired.str().c_str());

        // read mask image
        stringstream sstream_mask;
        sstream_mask<<m_data_path<<"mask/mask.png";
        filename_image_mask  = vpIoTools::path(sstream_mask.str().c_str());
        vpImageIo::read(this->m_desired_image, filename_read_image) ;

        m_visual_servo_tools.read_mask(filename_image_mask, m_mask_image);

        m_vs_rot.read_mask(filename_image_mask, m_mask_image);


        m_sId_first_rotation = new CFeatureLuminanceOmni();
//        m_sId_rotation =  new CFeatureLuminanceOmni();
//        m_sId_translation = new CFeatureLuminanceOmni();
//        m_sId_second_rotation = new CFeatureLuminanceOmni();

        initVisualServoFirstRotation();
    }


    //
    // visual servo on Vx and Vy to estimate theta*
    //
    void initVisualServoFirstRotation()
    {
//        m_logfile<<"init visual servo first rotation"<<endl;
//        m_vs_rot.init_visual_servo(m_sI_first_rotation, true, true, false, false, false, false);
//        m_vs_rot.build_desired_feature(m_desired_image, true, true, false, false, false, false,
//                                       m_sId_first_rotation, m_Lsd_first_rotation, m_Hsd_first_rotation, m_diagHsd_first_rotation);
//        m_logfile<<"ending init visual servo first rotation"<<endl;

    }

//    void initVisualServoTranslation()
//    {
//        m_vs_trans.init_visual_servo(m_sI_translation, true, false, false, false, false, true);
//        m_vs_trans.build_desired_feature(m_desired_image_rotated, true, false, false, false, false, true,
//                                         m_sId_translation, m_Lsd_translation, m_Hsd_translation, m_diagHsd_translation);
//    }



    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {
        m_iter++;
        m_iter_g++;

//        m_logfile<<"############################### m_iter"<<m_iter<<endl;

        double t = vpTime::measureTimeMs();
        //
        // acquire image
        //
        this->m_current_image = visp_bridge::toVispImage(*image);

        //        m_visual_servo_tools.bilateral_filter(m_current_image, m_current_image);
        m_visual_servo_tools.multiply_image_by_mask(m_current_image, m_mask_image);
        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

        //
        // first mode rotate robot until finding the correct angle
        //
        vpColVector v, e;

        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);

        vpImageTools::imageDifference(this->m_current_image,this->m_desired_image,this->m_difference_image) ;
        vpDisplay::display(this->m_difference_image);
        vpDisplay::flush(this->m_difference_image);

        stringstream sstream_filename;
        sstream_filename<<m_logs_path<<"current_images/current_image_"<<std::setw(6) << std::setfill('0')<<50*m_current_distance+m_iter<<".png";
        vpImageIo::write(m_current_image, sstream_filename.str().c_str());

        stringstream sstream_diff;
        sstream_diff<<m_logs_path<<"diff_images/diff_image_"<<std::setw(6) << std::setfill('0')<<50*m_current_distance+m_iter<<".png";
        vpImageIo::write(m_difference_image, sstream_diff.str().c_str());


        if (m_iter <= 50)
        {
//            this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_sId_first_rotation, m_current_image, m_iter,
//                                                                              m_lambda, m_mu, v,
//                                                                              m_Lsd_first_rotation, m_Hsd_first_rotation, m_diagHsd_first_rotation,
//                                                                              m_current_error, m_pred_error, m_logfile);
//            this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_current_image, v, m_logfile);

            m_required_angle = atan2(v[1], v[0]);
            if (m_required_angle > M_PI/2.0)
            {
                m_required_angle -= M_PI;
            }

            if (m_required_angle < -M_PI/2.0)
            {
                m_required_angle += M_PI ;
            }

            m_required_angle_degrees = (m_required_angle*180.0)/M_PI;
            m_logfile<<"########\tm_required_angles_degrees\t"<<m_required_angle_degrees<<"\titer"<<m_iter<<endl;
            m_log_angles<<50*m_current_distance+m_iter<<"\t"<<m_required_angle_degrees<<"\t"<<m_current_error<<"\t"<<v[0]<<"\t"<<v[1]<<endl;
        }

        else if ((m_iter < 95) && (m_iter>= 51))
        {
            ROS_INFO("Move ROBOT");
        }
        else
        {
            ROS_WARN("Motion ended");
            m_log_angles<<"########################################################"<<endl;
            m_iter = 0;
            m_current_distance++;
        }

//            m_visual_servo_tools.rotate_my_image(this->m_desired_image, m_center, m_required_angle_degrees, this->m_desired_image_rotated);

//            vpDisplay::display(m_desired_image_rotated);
//            vpDisplay::flush(m_desired_image_rotated);

            //
            // write rotated image
            //
//            stringstream str_rot_im;
//            str_rot_im<<m_logs_path<<"desired_images/desired_image_rotated.png";
//            vpImageIo::write(this->m_desired_image_rotated, str_rot_im.str().c_str());
            //            m_logfile<<"m_angle   "<<m_required_angle<<endl;
            //            if (m_required_angle<0.0) m_required_angle = 0.0;
//        }



    }

private:


    int m_current_distance;
    double m_sad;
    int m_nb_cycles;
    int m_nb_black_pixels;
    int m_index_desired_image_fwd;
    int m_index_desired_image_bwd;

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    //    ros::Publisher m_ros_pub_2;
    //    ros::Subscriber m_sonar_sub;
    ros::Subscriber m_ros_sub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_temp_desired_image;
    vpImage<unsigned char> m_desired_image_rotated;
    vpImage<unsigned char> m_new_desired_image_rotated;
    vpImage<unsigned char> m_desired_image_second_rotation;

    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_difference_image_translation;
    vpImage<unsigned char> m_difference_image_rotation;

    vpImage<unsigned char> m_mask_image;

    ofstream m_logfile;
    ofstream m_logerror;
    ofstream m_log_angles;

    string m_logs_path;
    string m_data_path;
    CFeatureLuminanceOmni m_sI_first_rotation ;
    CFeatureLuminanceOmni m_sI_translation ;
    CFeatureLuminanceOmni m_sI_rotation;
    CFeatureLuminanceOmni m_sI_second_rotation ;


    CFeatureLuminanceOmni *m_sId_translation;
    CFeatureLuminanceOmni *m_sId_first_rotation;
    CFeatureLuminanceOmni *m_sId_second_rotation;
    CFeatureLuminanceOmni *m_sId_rotation;

    vpMatrix m_Lsd_first_rotation, m_Lsd_second_rotation, m_Lsd_translation, m_Lsd_rotation;
    vpMatrix m_Hsd_first_rotation, m_Hsd_second_rotation, m_Hsd_translation, m_Hsd_rotation;
    vpMatrix m_diagHsd_first_rotation, m_diagHsd_second_rotation, m_diagHsd_translation, m_diagHsd_rotation;

    int m_nb_croissance;

    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    int m_iter_g;
    geometry_msgs::Twist m_velocity;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_desired_image_rot;
    vpDisplayX m_display_error;
    vpDisplayX m_display_error_translation;
    vpDisplayX m_display_error_rotation;

    double m_current_error;
    double m_pred_error;
    double m_initial_error;
    double m_mu;
    double m_lambda;
    int m_first_forward;
    int m_first_backward;
    int m_nb_tours;
    double m_threshold_error;
    double m_diff_error;
    double m_mean_diff_error;
    double m_required_angle;
    double m_required_angle_degrees;

    double m_new_required_angle;
    double m_new_required_angle_degrees;

    double m_odometry_angle;

    double m_lambda_rotation;

    bool m_first_rotation_mode;
    bool m_second_rotation_mode;
    bool m_translation_mode;
    bool m_combined_translation_and_rotation;

    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_center[2];
    VisualServoTools m_visual_servo_tools;
    VisualServoTools m_vs_trans;
    VisualServoTools m_vs_rot;

};

