#include "VisualServoTools.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <image_transport/image_transport.h>
#include "visp_bridge/image.h"
#include <tf/transform_broadcaster.h>

//static const char WINDOW[] = "Image window";

using namespace std;

class VCompass
{

public:

    VCompass()
        : m_it(m_nh)
    {
        m_iter_g                            = 0;
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

        m_display_current_image.init(m_current_image, 0, 0, "Current image");

        m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
        m_display_desired_image_rot.init(m_desired_image_rotated, m_width+10, m_height, "Desired image rotated");

        m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");
        m_display_error_translation.init(m_difference_image_translation, 0, m_height, "Difference translation");
        m_display_error_rotation.init(m_difference_image_rotation, 2*m_width+10, m_height, "Difference rotation 2");

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

        m_sId_first_rotation = new CFeatureLuminanceOmni();
        m_sId_rotation =  new CFeatureLuminanceOmni();
        m_sId_translation = new CFeatureLuminanceOmni();
        m_sId_second_rotation = new CFeatureLuminanceOmni();

        initVisualServoFirstRotation();
    }


    //
    // visual servo on Vx and Vy to estimate theta*
    //
    void initVisualServoFirstRotation()
    {
        m_visual_servo_tools.init_visual_servo(m_sI_first_rotation, true, true, false, false, false, false);
        m_visual_servo_tools.build_desired_feature(m_desired_image,true, true, false, false, false, false
                                                  /* m_sId_first_rotation, m_Lsd_first_rotation, m_Hsd_first_rotation, m_diagHsd_first_rotation*/);
    }

    //
    // visual servo on Vx and Vy for a better estimate of
    //
    void initVisualServoRotation(vpImage<unsigned char>& desired_image)
    {
//        m_visual_servo_tools.init_visual_servo(m_sI_rotation, true, true, false, false, false, false);
        m_visual_servo_tools.build_desired_feature(desired_image, true,  true, false, false, false, false
                                                   /*m_sId_rotation, m_Lsd_rotation, m_Hsd_rotation, m_diagHsd_rotation*/);

    }

    void initVisualServoTranslation()
    {
        m_visual_servo_tools.init_visual_servo(m_sI_translation, true, false, false, false, false, true);
        m_visual_servo_tools.build_desired_feature(m_desired_image_rotated, true, false, false, false, false, true
                                                   /*m_sId_translation, m_Lsd_translation, m_Hsd_translation, m_diagHsd_translation*/);
    }

//    void initVisualServoSecondRotation()
//    {

//        m_visual_servo_tools.init_visual_servo(m_sI_second_rotation, true, true, false, false, false, false);
//        m_visual_servo_tools.build_desired_feature(m_desired_image_second_rotation,true, true, false, false, false, false,
//                                                   m_sId_second_rotation, m_Lsd_second_rotation, m_Hsd_second_rotation, m_diagHsd_second_rotation);
//    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {
        m_iter++;
        m_iter_g++;

        m_logfile<<"############################### m_iter"<<m_iter<<endl;

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

        if (m_first_rotation_mode)
        {
            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);

            vpImageTools::imageDifference(this->m_current_image,this->m_desired_image,this->m_difference_image) ;
            vpDisplay::display(this->m_difference_image);
            vpDisplay::flush(this->m_difference_image);
            m_logfile<<"### entering first rotation mode"<<endl;

            stringstream sstream_filename;
            sstream_filename<<m_logs_path<<"current_images/current_image_first_rotation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_current_image, sstream_filename.str().c_str());

            stringstream sstream_diff;
            sstream_diff<<m_logs_path<<"diff_images/diff_image_first_rotation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_difference_image, sstream_diff.str().c_str());

            if (m_iter == 1)
            {
//                this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_sId_first_rotation, m_current_image, m_iter,
//                                                                                  m_lambda, m_mu, v, m_Lsd_first_rotation, m_Hsd_first_rotation, m_diagHsd_first_rotation,
//                                                                                  m_current_error, m_pred_error, m_logfile);
                this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_current_image,
                                                                                   v, /*m_Lsd_first_rotation, m_Hsd_first_rotation, m_diagHsd_first_rotation,*/
                                                                                   m_logfile);

                m_required_angle = atan2(v[1], v[0]);

                if (m_required_angle > M_PI/2.0 )
                {
                    m_required_angle -= M_PI;
                }

                if (m_required_angle < -M_PI/2.0 )
                {
                    m_required_angle += M_PI ;
                }

                m_required_angle_degrees = (m_required_angle*180.0)/M_PI;
//                m_logfile<<"m_required_angles_degrees"<<m_required_angle_degrees<<endl;
                m_log_angles<<m_iter<<"\t"<<m_required_angle_degrees<<endl;
                m_visual_servo_tools.rotate_my_image(this->m_desired_image, m_center, m_required_angle_degrees, this->m_desired_image_rotated);
                vpDisplay::display(m_desired_image_rotated);
                vpDisplay::flush(m_desired_image_rotated);

//                ROS_DEBUG("#### REQUIRED_ANGLE", m_required_angle);
                //
                // write rotated image
                //
                stringstream str_rot_im;
                str_rot_im<<m_logs_path<<"desired_images/desired_image_rotated.png";
                vpImageIo::write(this->m_desired_image_rotated, str_rot_im.str().c_str());
                //            m_logfile<<"m_angle   "<<m_required_angle<<endl;
                //            if (m_required_angle<0.0) m_required_angle = 0.0;
            }

            //
            // compute theta by visual compass
            //

            int step  = 2;
            double theta_degrees, theta_radians,omega;
            double new_required_angle;
            double lambda = 0.5;

            //
            // compute angle between current and desired image until difference is almost zero
            //
            if (m_required_angle >= 0)
            {
                theta_degrees  = this->m_visual_servo_tools.find_best_angle(m_current_image, m_desired_image , m_center, step, m_logfile);
                theta_radians = theta_degrees*M_PI/180.0;
                m_logfile<<"Radians : required\t"<<m_required_angle<<"\tvisual compass:\t"<<theta_radians<<endl;
                m_logfile<<"Degrees : required\t"<<m_required_angle_degrees<<"\tvisual_compass:\t"<<theta_degrees<<endl;
                omega = -lambda*(theta_radians - m_required_angle);

                m_velocity.linear.x = 0.0;
                m_logfile<<"velocity\t"<<v<<endl;
                m_logfile<<"angular velocity"<<omega<<endl;
            }

            if (m_required_angle < 0)
            {
                m_new_required_angle = fabs(m_required_angle);
                m_new_required_angle_degrees = (m_new_required_angle*180.0)/M_PI;

                theta_degrees  = this->m_visual_servo_tools.find_best_angle(m_current_image, m_desired_image, m_center, step, m_logfile);
                //                theta_degrees = theta_degrees - 180.0;
                theta_radians = theta_degrees*M_PI/180.0;

                m_logfile<<"Radians : required\t"<<m_new_required_angle<<"\tvisual compass:\t"<<theta_radians<<endl;
                m_logfile<<"Degrees : required\t"<<m_new_required_angle_degrees<<"\tvisual_compass:\t"<<theta_degrees<<endl;
                omega = lambda*(theta_radians - m_new_required_angle);

                m_velocity.linear.x = 0.0;
                m_logfile<<"velocity\t"<<v<<endl;
                m_logfile<<"angular velocity"<<omega<<endl;

            }

            if (fabs(omega) < 2e-2)
            {
                m_logfile<<"omega is almost zero"<<endl;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                m_first_rotation_mode = false;
                m_combined_translation_and_rotation = true;
                //                m_translation_mode = true;
//                this->initVisualServoTranslation();
                m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                m_visual_servo_tools.initialize();
                ros::spinOnce();
            }
            else
            {
                if (fabs(omega) > 0.1)
                {
                    omega = vpMath::sign(omega)*0.1;
                }
                m_logfile<<"######## in else"<<endl;
                m_velocity.angular.z = omega;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
        }

       else if (m_translation_mode)
        {
            stringstream sstream_filename;
            sstream_filename<<m_logs_path<<"current_images/current_image_translation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_current_image, sstream_filename.str().c_str());


            stringstream sstream_diff;
            sstream_diff<<m_logs_path<<"diff_images/diff_image_translation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_difference_image, sstream_diff.str().c_str());


            m_logfile<<"##### entering translation mode"<<endl;
            vpImageTools::imageDifference(this->m_current_image,this->m_desired_image_rotated,this->m_difference_image_translation) ;
            vpDisplay::display(this->m_difference_image_translation);
            vpDisplay::flush(this->m_difference_image_translation);

//            this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_translation, m_sId_translation, m_current_image, m_iter,
//                                                                              m_lambda, m_mu, v, m_Lsd_translation, m_Hsd_translation, m_diagHsd_translation,
//                                                                              m_current_error, m_pred_error, m_logfile);

            this->m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI_translation,/* m_sId_translation,*/ m_current_image,
                                                                               v, /*m_Lsd_translation, m_Hsd_translation, m_diagHsd_translation,*/
                                                                               m_logfile);

            m_logfile<<"v[0]"<<v[0]<<"\tv[1]"<<v[1]<<endl;

            if (fabs(v[0]) > 0.2)
            {
                v[0] = vpMath::sign(v[0])* 0.2;
            }

            if (fabs(v[1]) > 0.2)
            {
                v[1]  = vpMath::sign(v[1])*0.2;
            }

            m_velocity.linear.x = v[0];
            m_velocity.angular.z = v[1];

            if (fabs(m_current_error - m_pred_error)< 2e-6)
                //            if (fabs(m_current_error - m_pred_error)< 2e-7)
            {
                m_logfile<<"linear velocity is almost zero"<<endl;
                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_translation_mode = false;
                this->m_second_rotation_mode = true;

                m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                m_visual_servo_tools.initialize();
                ros::spinOnce();
            }
            else
            {
                m_logfile<<"### velocity is not null"<<v<<endl;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
        }

        else if (m_second_rotation_mode)
        {
            stringstream sstream_filename;
            sstream_filename<<m_logs_path<<"current_images/current_image_second_rotation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_current_image, sstream_filename.str().c_str());

            stringstream sstream_diff;
            sstream_diff<<m_logs_path<<"diff_images/diff_image_second_rotation_mode_"<<m_iter<<".png";
            vpImageIo::write(m_difference_image, sstream_diff.str().c_str());

            //
            // display first image rotation
            //
            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);
            //
            //  display diff image
            //
            vpImageTools::imageDifference(this->m_current_image,this->m_desired_image,this->m_difference_image_rotation) ;
            vpDisplay::display(m_difference_image_rotation);
            vpDisplay::flush(m_difference_image_rotation);

            double required_angle_degrees = m_required_angle*M_PI/180.0;
            int step  = 2;

            m_logfile<<"### entering second rotation mode"<<endl;
            double theta_prime_degrees = 0.0;
            theta_prime_degrees = this->m_visual_servo_tools.find_best_angle(m_current_image, this->m_desired_image,
                                                                             m_center, step, m_logfile);

            double theta_prime_radians = theta_prime_degrees * M_PI/180.0;

            double lambda_rot = 0.5;
            double omega;
            double new_lambda = -lambda_rot;
            if (m_required_angle < 0)
            {
                omega = -new_lambda*(theta_prime_radians);
            }

            else
            {
                omega = -lambda_rot*(theta_prime_radians);
            }

//            m_nb_black_pixels = 0;
            //                    if ((int)m_difference_image_rotation(i,j) == 128)
            //                    {
            //                        m_nb_black_pixels++;
            //                    }


//            m_sad = 999.0;

            for (int i = 0; i< m_width; i++)
            {
                for (int j = 0; j < m_height; j++)
                {
//                    m_logfile<<"diff_image_pixels\t"<<(int)m_difference_image_rotation(i,j)<<endl;
                    m_sad += abs(m_difference_image_rotation(i,j));
                }
            }

            m_sad /=(m_width*m_height);
            m_logfile<<"m_sad"<<m_sad<<endl;

            m_velocity.linear.x = 0.0;
//            m_logfile<<"m_nb_pixels"<<m_width*m_height*3/4<<endl;
            m_logfile<<"Radians : required\t"<<m_required_angle<<"\tvisual compass:\t"<<theta_prime_radians<<endl;
            m_logfile<<"Degrees : required\t"<<m_required_angle * 180.0/M_PI<<"\tvisual_compass:\t"<<theta_prime_degrees<<endl;
            m_logfile<<"velocity\t"<<v<<endl;
            m_logfile<<"angular velocity"<<omega<<endl;
            //            if (fabs(omega)< 2e-2)
            if (fabs(theta_prime_degrees ) <= 2.0)
            {
                m_logfile<<"omega is almost zero"<<endl;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_second_rotation_mode = false;
//                this->m_first_rotation_mode = true;
                m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                m_visual_servo_tools.initialize();
                initVisualServoFirstRotation();
                ros::spinOnce();
            }

            else
            {
                if (fabs(omega) > 0.1)
                {
                    omega = vpMath::sign(omega)*0.1;
                }
                m_logfile<<"######## in else"<<endl;
                m_velocity.angular.z = omega;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
        }
    }

private:


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
};

