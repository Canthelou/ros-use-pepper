#include "VisualServoTools.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <image_transport/image_transport.h>
#include "visp_bridge/image.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>


//#ifndef DI
//#define DI 50
//#endif

//#ifndef DJ
//#define DJ 50
//#endif

//#ifndef NBRI
//#define NBRI 250 //596/2
//#endif

//#ifndef NBRJ
//#define NBRJ 250 //596/2
//#endif
//#ifndef PAS
//#define PAS 1
//#endif


//#define REPTYPE CFeatureLuminanceOmni::CARTESIANSPHERICAL
//#define GRAPCALCTYPE CFeatureLuminanceOmni::DIRECT
//#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_NEARESTNEIGH

//#define CURRENT
//#define INDICATORS
//#define LISTINITPOSES
//#define CSTRHO

using namespace std;

class VCompass
{

public:

    VCompass()
        : m_it(m_nh)
    {

        m_iter                              = -1;
        m_nb_cycles                         = 0;
        m_nb_tours                          = 0;
        m_mean_diff_error                   = 0.0;
        m_diff_error                        = 0.0;
        m_first_forward                     = 0;
        m_first_backward                    = 0;
        m_width                             = m_visual_servo_tools.getWidth();
        m_height                            = m_visual_servo_tools.getHeight();

        m_first_rotation_mode               = true;
        m_second_rotation_mode              = false;
        m_combined_translation_and_rotation = false;

        m_required_angle                    = 0.0;
        m_required_angle_degrees            = 0.0;

        m_nb_croissance                     = 0;
        m_index_desired_image_fwd           = 1;
        m_index_desired_image_bwd           = -1;

        m_counter                           = 0;
        m_px                                = m_visual_servo_tools.getPx();
        m_py                                = m_visual_servo_tools.getPy();
        m_u0                                = m_visual_servo_tools.getU0();
        m_v0                                = m_visual_servo_tools.getV0();
        m_xi                                = m_visual_servo_tools.getXi();
        m_center[0]                         = m_u0;
        m_center[1]                         = m_v0;

        m_center_image[0]                   = m_height/2;
        m_center_image[1]                   = m_width/2;

        m_mean_required_angles              = 0.0;

        this->m_prev_vel.init();
        this->m_curr_vel.init();


        string cameraTopic, robotTopic, visualServoMovements, poseTopic;
        m_nh.param("cameraTopic", cameraTopic, string(""));
        m_nh.param("robotTopic", robotTopic, string(""));
        m_nh.param("poseTopic", poseTopic, string(""));
        m_nh.param("visualServoMovements", visualServoMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path, string(""));
        m_nh.param("error_threshold", m_threshold_error, double(0.0));
        m_nh.param("lambda", m_lambda, double(10.0));
        m_nh.param("mu", m_mu, double(10.0));
        m_nh.param("lambda_rot", m_lambda_rot, double(0.0));

        m_nh.getParam("poseTopic", poseTopic);
        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("visualServoMovements", visualServoMovements);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        m_nh.getParam("error_threshold",m_threshold_error);
        m_nh.getParam("robotTopic", robotTopic);
        m_nh.getParam("lambda", m_lambda);
        m_nh.getParam("mu", m_mu);
        m_nh.getParam("lambda_rot", m_lambda_rot);


        m_vs_first_rot = new VisualServoTools();
        m_vs_translation = new VisualServoTools();


        //        this->initValues();
        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        stringstream str_error;
        str_error<<m_logs_path<<"error.mat";
        m_logerror.open(str_error.str().c_str());

        stringstream str_angles;
        str_angles<<m_logs_path<<"angles.mat";
        m_log_angles.open(str_angles.str().c_str());


        stringstream str_omega;
        str_omega<<m_logs_path<<"omega.mat";
        m_log_omega.open(str_omega.str().c_str());

        m_current_image.init(m_height, m_width);
        m_current_image_rerotated.init(m_height, m_width);
        m_desired_image.init(m_height, m_width);
        m_difference_image.init(m_height, m_width);
        m_desired_image_rotated.init(m_height, m_width);
        m_new_desired_image_rotated.init(m_height, m_width);

        m_temp_curr.init(m_height, m_width);
        m_temp_des.init(m_height, m_width);

        m_difference_image_translation.init(m_height, m_width);
        m_difference_image_rotation.init(m_height, m_width);
        //        m_temp_desired_image.init(m_height, m_width);
        m_desired_image_rotated_rerotated.init(m_height, m_width);

        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        //        m_display_temp_curr.init(m_temp_curr,0,0,"temp_curr");

        m_display_desired_image.init(m_desired_image, m_width+70, 0, "Desired image");
        //        m_display_temp_des.init(m_temp_des, m_width+70, 0, "temp_des");

        //
        m_display_desired_image_rot.init(m_desired_image_rotated, 2*m_width+70, 0, "Desired image rotated");

        //        m_display_current_image_rerotated.init(m_current_image_rerotated, 0, m_height+60,"Current image rerotated");
        //        m_display_desired_image_rot_rerot.init(m_desired_image_rotated_rerotated, m_width+70, m_height+60, "Desired image rotated rerotated");
        m_display_error_rotation.init(m_difference_image_translation, 2*m_width+70, m_height+60, "Difference translation");

        m_display_error.init(m_difference_image_rotation, 3*m_width+70, 0, "Difference");


        //        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        //        m_display_desired_image.init(m_desired_image, m_width+70, 0, "Desired image");
        //        m_display_desired_image_rot.init(m_desired_image_rotated, 2*m_width+70, 0, "Desired image rotated");

        //        m_display_current_image_rerotated.init(m_current_image_rerotated, 0, m_height+60,"Current image rerotated");
        //        m_display_desired_image_rot_rerot.init(m_desired_image_rotated_rerotated, m_width+70, m_height+60, "Desired image rotated rerotated");
        //        m_display_error_rotation.init(m_difference_image_translation, 2*m_width+70, m_height+60, "Difference translation");

        //        m_display_error.init(m_difference_image_rotation, 3*m_width+70, 0, "Difference");

        //                m_display_desired_image_rot.init(m_new_desired_image_rotated, m_width+30, m_height, "New Desired Image Oriented");
        //        m_display_error_translation.init(m_difference_image_translation, 0, m_height, "Difference translation");
        //        m_display_error_rotation.init(m_difference_image_rotation, 2*m_width+30, m_height, "Difference rotation 2");
        m_mask_image.init(m_height, m_width);



//        m_logfile<<"finding best angle"<<endl;

//        string filename_image_1 = "/home/yalj/catkin_ws/src/my_package/logs/tmp_images/current_image0002.png";
//        string filename_masque_image = "/home/yalj/catkin_ws/src/my_package/logs/tmp_images/masque.png";

//        vpImage<unsigned char> image_in_1, mask;
//        vpImageIo::read(image_in_1, filename_image_1);
//        vpImageIo::read(mask, filename_masque_image);


//        m_visual_servo_tools.set_mask(mask);
//        m_visual_servo_tools.multiply_image_by_mask(image_in_1,mask);
//        m_visual_servo_tools.find_best_angle(image_in_1, image_in_1, m_center, 0.48, m_logerror);




        m_image_sub = m_it.subscribe(cameraTopic, 1, &VCompass::imageCallback, this);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1);

        m_cam_param = CCameraOmniParameters(m_px, m_py, m_u0, m_v0, m_xi);
        //        m_vs_translation = new VisualServoTools();
        //        m_vs_translation = new VisualServoTools();

        initVCompass();
    }

    ~VCompass()
    {
        //        delete m_vs_first_rot;
        m_logfile.close();

    }

    //    void initValues()
    //    {
    ////        this->m_current_error = 9999;
    ////        m_iter      = 0;
    ////        //        m_mu        = 0.01;
    ////        //m_lambda  = 50 ;
    ////        m_nh.getParam("lambda", m_lambda);
    ////        m_nh.getParam("mu", m_mu);

    ////        m_nb_croissance = 0;
    ////        m_mean_diff_error = 0.0;
    ////        m_diff_error = 0.0;
    //    }


    void initVCompass()
    {

        m_logfile<<"init VCompass"<<endl;
        std::string filename_read_image, filename_image_mask;

        // read desired image
        stringstream sstream_desired;
        sstream_desired<<m_logs_path<<"desired_images/desired_image.png";
        filename_read_image = vpIoTools::path(sstream_desired.str().c_str());

        // read mask image
        stringstream sstream_mask;
        sstream_mask<<m_data_path<<"mask/mask.png";
        filename_image_mask  = vpIoTools::path(sstream_mask.str().c_str());

        vpImage<unsigned char> tmp_image_des, tmp_mask;
        vpImageIo::read(tmp_image_des, filename_read_image) ;
        tmp_image_des.halfSizeImage(m_desired_image);

        m_visual_servo_tools.read_mask(filename_image_mask, tmp_mask);
        tmp_mask.halfSizeImage(m_mask_image);

//        this->m_visual_servo_tools.equalizeHisto(m_desired_image, m_desired_image);
//        this->m_visual_servo_tools.bilateral_filter(m_desired_image, m_desired_image);
        m_visual_servo_tools.multiply_image_by_mask(m_desired_image, m_mask_image);

        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);
        //        m_vs_translation->rotate_my_image(this->m_desired_image, m_center, 90, this->m_desired_image_rotated);
        //        this->m_visual_servo_tools.canny(m_desired_image, m_desired_image);

        stringstream sstream_desired_modif;
        sstream_desired_modif<<m_logs_path<<"desired_images/desired_image_modif.png";
        vpImageIo::write(m_desired_image, sstream_desired_modif.str().c_str());

        m_vs_first_rot->set_mask(m_mask_image);
        m_vs_first_rot->set_lambda(m_lambda);
        m_vs_first_rot->set_mu(m_mu);
        m_vs_first_rot->initialize();

        m_logfile<<"###### m_lambda"<<m_lambda<<"\tm_mu"<<m_mu<<endl;

        m_vs_translation->set_mask(m_mask_image);
        m_vs_translation->set_lambda(m_lambda);
        m_vs_translation->set_mu(m_mu);
        m_vs_translation->initialize();


    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {

        //        m_velocity.angular.z = 0.1;
        //        m_ros_pub.publish(m_velocity);
        //        ros::spinOnce();
        //        return;

        m_logfile<<"in image callback"<<endl;
        m_iter_g++;


        //        stringstream sstream_current_image;
        //        sstream_current_image<<m_logs_path<<"current_images/current_image"<<std::setw(4)<<std::setfill('0')<<m_iter_g<<".png";
        //        vpImage<unsigned char> curr_im(m_height, m_width);
        //        curr_im = visp_bridge::toVispImage(*image);
        //        // read mask image
        //        stringstream sstream_mask;
        //        sstream_mask<<m_data_path<<"mask/mask.png";
        //        string filename_image_mask  = vpIoTools::path(sstream_mask.str().c_str());
        //        vpImage<unsigned char> tmp_mask;
        //        m_visual_servo_tools.read_mask(filename_image_mask, tmp_mask);
        //        m_visual_servo_tools.multiply_image_by_mask(curr_im, tmp_mask);
        //        vpImageIo::write(curr_im, sstream_current_image.str().c_str());
        //        m_velocity.linear.x = 0.1;
        //        m_ros_pub.publish(m_velocity);
        //        ros::spinOnce();
        //        return;

        //        ros::Rate r(2);
        //        while(ros::ok())
        //        {
        //            if (m_iter_g == 1)
        //            {
        //                m_velocity.linear.x = 0.5;
        //                m_ros_pub.publish(m_velocity);
        //                ros::spinOnce();
        //            }
        //            r.sleep();
        //            if (m_iter_g == 2)
        //            {
        //                m_velocity.linear.x = 0.0;
        //                m_ros_pub.publish(m_velocity);
        //                ros::spinOnce();
        //            }
        //        }
        //        return;





//        if (m_iter_g == 1)
//        {
//        }
        return;

        //
        // acquire image
        //
        vpImage<unsigned char> curr_im_temp(m_height, m_width);

        curr_im_temp = visp_bridge::toVispImage(*image);
        curr_im_temp.halfSizeImage(m_current_image);

//        this->m_visual_servo_tools.equalizeHisto(m_current_image, m_current_image);
//        this->m_visual_servo_tools.bilateral_filter(m_current_image, m_current_image);

        this->m_visual_servo_tools.multiply_image_by_mask(m_current_image, m_mask_image);

        //        this->m_visual_servo_tools.canny(m_current_image, m_current_image);
        //        this->m_visual_servo_tools.denoising(m_current_image, m_current_image);
        //        m_visual_servo_tools.rotate_my_image(m_current_image, m_center_image, -90.0, m_current_image);

        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);


        //
        // first mode rotate robot until finding the correct angle
        //

        vpColVector v, e;

        //        vpImage<unsigned char> m_temp_desired_image_oriented(m_height, m_width);
        //        vpImage <unsigned char> m_temp_desired_image(m_height, m_width);

        if (m_first_rotation_mode)
        {


            m_vs_first_rot->increment_iteration();
            // find best angle
            m_sId = new CFeatureLuminanceOmni();

            double angle = m_vs_first_rot->find_best_angle(m_desired_image, m_current_image, this->m_center, 2, m_logerror);
            m_logfile<<"## estimated angle\t"<<angle<<endl;
            m_vs_first_rot->rotate_my_image(m_desired_image, m_center, angle, m_desired_image_rotated);
            stringstream sstream_desired_rot, sstream_curr_rot;

            sstream_desired_rot<<m_logs_path<<"desired_images/desired_image_first_rot.png";
            vpImageIo::write(m_desired_image_rotated, sstream_desired_rot.str().c_str());

            sstream_curr_rot<<m_logs_path<<"current_images/current_image_first_rot"<<std::setw(4) << std::setfill('0')<<m_vs_first_rot->get_iteration_number()<<".png";
            vpImageIo::write(m_current_image, sstream_curr_rot.str().c_str());


            // rotate the desired image comparing to the current image
            m_vs_first_rot->init_visual_servo(m_sI_first_rotation, false, true, false, false, false, false, m_logfile);

            m_vs_first_rot->build_desired_feature(m_desired_image_rotated, false, true, false, false, false, false,
                                                  m_sId, m_Lsd, m_Hsd, m_diagHsd);




            //             m_logfile<<"### desired features are built"<<endl;
            //             m_vs_first_rot->perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_sId, m_current_image,
            //                                                                    v, m_Lsd, m_Hsd, m_diagHsd, m_logfile);

            //
            // Perform visual servoing iteration
            //
            m_sI_first_rotation.buildFrom(m_current_image);

            // Minimisation
            vpMatrix H;
            vpColVector v, e, error;


            vpRobust robust(0);
            robust.setThreshold(0.0);
            vpColVector w;
            vpColVector weighted_error;

            // compute current error
            m_sI_first_rotation.error(*m_sId,error);

            //        cout<<"#### error"<<error<<endl;
            w.resize(error.getRows());
            weighted_error.resize(error.getRows());
            w = 1;
            robust.MEstimator(vpRobust::TUKEY, error, w);

            H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();

            vpMatrix interaction_matrix ((m_Lsd).getRows(),(m_Lsd).getCols());
            int nb_total_points = (m_Lsd).getRows();

            for (int i=0; i<nb_total_points; i++)
            {
                weighted_error[i] = w[i]*error[i];
                interaction_matrix[i][0] = m_Lsd[i][0]*w[i];
//                interaction_matrix[i][1] = m_Lsd[i][1]*w[i];
            }


            //	compute the control law
            e = H * interaction_matrix.t() * weighted_error;

            double normError  = weighted_error.sumSquare();

            if (m_vs_first_rot->get_iteration_number() > 0)
            {
                m_prev_vel_first_rot = m_curr_vel_first_rot;
                m_logfile<<"m_prev_vel"<<m_prev_vel_first_rot<<endl;
            }
            m_curr_vel_first_rot = - m_lambda*e;
            m_logfile<<"#### velocity"<<m_curr_vel_first_rot<<endl;
            m_logfile<<"### V_y\t"<<m_curr_vel_first_rot<<"iteration\t"<<m_vs_first_rot->get_iteration_number()<<endl;

            if (m_vs_first_rot->get_iteration_number()>0)
            {
                m_logfile<<"### signs"<<vpMath::sign(m_prev_vel_first_rot[0])*vpMath::sign(m_curr_vel_first_rot[0])<<endl;
            }

            double lambda = 0.25;
            //            double omega = lambda*m_curr_vel_first_rot[1]*vpMath::sign(m_curr_vel_first_rot[0]);

            double omega = lambda*m_curr_vel_first_rot[0];

            //            m_sId->staticClean();
            m_vs_first_rot->clean();
            delete m_sId;

            //            if ((fabs(v[0])< 5e-2) && (m_vs_first_rot->get_iteration_number()>100))
            //            if (v[0] < 0.0)

            m_logfile<<"m_vs_first_rotation"<<m_vs_first_rot->get_iteration_number()<<endl;
            //            if (angle  == 134 )

            if (m_vs_first_rot->get_iteration_number()>2)
//            if (m_vs_first_rot->get_iteration_number()> 37)
//            if (fabs(omega) < 1e-3 )
//            {
//                if (vpMath::sign(m_prev_vel_first_rot[0])*vpMath::sign(m_curr_vel_first_rot [0]) == -1)
            {
                    m_velocity.angular.z = 0.0;
                    m_ros_pub.publish(m_velocity);
                    m_first_rotation_mode = false;
                    m_combined_translation_and_rotation = true;

                    m_logfile<<endl;
                    m_logfile<<endl;
                    m_logfile<<"####### Entering rotation and translation meode"<<endl;
                    ros::spinOnce();
//                }

//                else
//                {

//                    m_velocity.angular.z = omega;
//                    m_ros_pub.publish(m_velocity);
//                    ros::spinOnce();
//                }
            }
            else
            {
                m_velocity.angular.z = omega;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }

            /*
            m_logfile<<"### Entering first rotation mode"<<endl;
            m_vs_first_rot->increment_iteration();

            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);

            vpImageTools::imageDifference(this->m_current_image, this->m_desired_image, m_difference_image) ;

            vpDisplay::display(this->m_difference_image);
            vpDisplay::flush(this->m_difference_image);

            stringstream sstream_diff;
            sstream_diff<<m_logs_path<<"diff_images/diff_image_first_rotation_mode_"<<m_vs_first_rot->get_iteration_number()<<".png";
            vpImageIo::write(m_difference_image, sstream_diff.str().c_str());
            stringstream sstream_filename;
            sstream_filename<<m_logs_path<<"current_images/current_image_first_rotation_mode_"<<std::setw(4) << std::setfill('0')<<m_vs_first_rot->get_iteration_number()<<".png";
            vpImageIo::write(m_current_image, sstream_filename.str().c_str());



//            if (m_vs_first_rot->get_iteration_number() < 20)
//            {
                m_logfile<<"estimation of angle"<<endl;


                m_vs_first_rot->init_visual_servo(m_sI_first_rotation, true, true, false, false, false, false, m_logfile);
                //                m_vs_first_rot->build_desired_feature(m_desired_image, true, true, false, false, false, false, m_logfile);

                m_vs_first_rot->build_desired_feature(m_desired_image, true, true, false, false, false, false, m_sId, m_Lsd, m_Hsd, m_diagHsd);

                m_logfile<<"### desired features are built"<<endl;
                m_vs_first_rot->perform_a_VS_iteration_with_MEstimator(m_sI_first_rotation, m_sId, m_current_image,
                                                                       v, m_Lsd, m_Hsd, m_diagHsd, m_logfile);
                //                v = v/sqrt(vpMath::sqr(v[0]) + vpMath::sqr(v[1]));
                m_logfile<<"############ velocity\t"<<v<<endl;

                m_required_angle = atan2(v[1], v[0]);




                if (m_required_angle > M_PI/2.0)
                {
                    m_required_angle -= M_PI;
                }

                if (m_required_angle < -M_PI/2.0)
                {
                    m_required_angle += M_PI ;
                }

                m_required_angle_degrees = vpMath::deg(m_required_angle);

                m_angles.push_back(m_required_angle_degrees);
                //                m_mean_required_angles += m_required_angle_degrees;
                //                double mean  = m_mean_required_angles/(m_vs_first_rot->get_iteration_number()+1);




                m_logfile<<"######### m_required_angle atan2\t"<<m_required_angle_degrees<<endl;
                m_log_angles<<m_vs_first_rot->get_iteration_number()<<"\t"<<m_required_angle_degrees<<endl;
                //                m_log_angles<<m_vs_first_rot->get_iteration_number()<<"\t"<<0<<"\t"<<v[0]<<"\t"<<v[1]<<endl;
                //                m_logfile<<"######### m_mean_required_angle atan2\t"<<mean<<endl;
                //                m_log_angles<<this->m_vs_first_rot->get_iteration_number()<<"\t"<<m_required_angle_degrees<<endl;
                m_initial_image = m_current_image;

//            }




            else if (m_vs_first_rot->get_iteration_number() == m_angles.size())
            {
                return;

                m_logfile<<"computing the median"<<endl;

                std::sort(m_angles.begin(), m_angles.end());
                m_required_angle_degrees = (m_angles[m_angles.size()/2 -1] + m_angles[m_angles.size()/2])/2.0;
                m_required_angle = vpMath::rad(m_required_angle_degrees);
                m_logfile<<"m_required_angle_degrees"<<m_required_angle_degrees<<endl;
            }
            else
            {
                //
                // compute theta by visual compass
                //
                int step  = 2;
                double theta_degrees, theta_radians, omega;
                double lambda = 0.5;

                //
                // compute angle between current and desired image until difference is almost zero
                //

                theta_degrees  = this->m_visual_servo_tools.find_best_angle(m_initial_image, m_current_image, m_center, step, m_logfile);

                theta_radians = vpMath::rad(theta_degrees);
                m_logfile<<"Radians : required\t"<<m_required_angle<<"\tvisual compass:\t"<<theta_radians<<endl;
                m_logfile<<"Degrees : required\t"<<m_required_angle_degrees<<"\tvisual_compass:\t"<<theta_degrees<<endl;

                omega = -lambda*(theta_radians - m_required_angle);

                m_velocity.linear.x = 0.0;
                m_logfile<<"velocity\t"<<v<<endl;
                m_logfile<<"angular velocity"<<omega<<endl;

                //            m_vs_first_rot.clean();

                if (fabs(omega)< 2e-2)
                    //                if (fabs(theta_degrees - m_required_angle_degrees) < 1.)
                {
                    m_logfile<<"omega is almost zero"<<endl;
                    m_velocity.angular.z = 0.0;
                    m_ros_pub.publish(m_velocity);
                    m_first_rotation_mode = false;
                    m_combined_translation_and_rotation = true;
                    ros::spinOnce();
                }
                else
                {
                    //                if (fabs(omega) > 0.1)
                    //                {
                    //                    omega = vpMath::sign(omega)*0.1;
                    //                }
                    m_logfile<<"######## in else"<<endl;
                    m_velocity.angular.z = omega;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }
            }
*/


        }


        else if (m_combined_translation_and_rotation)
        {

            double time_transl = vpTime::measureTimeMs();
            /*

            m_sId = new CFeatureLuminanceOmni();
            m_vs_translation->increment_iteration();

            m_vs_translation->init_visual_servo(this->m_sI_translation, true, false, false, false, false, true);
            m_vs_translation->build_desired_feature(m_desired_image_rotated, true, false, false, false, false, true, m_sId, m_Lsd, m_Hsd, m_diagHsd);

            m_logfile<<"######### BEGIN TRANSLATION #########"<<endl;

            this->m_vs_translation->perform_a_VS_iteration_with_MEstimator(this->m_sI_translation, m_sId, m_current_image,
                                                                           m_curr_vel, m_Lsd, m_Hsd, m_diagHsd, m_logfile);

//            m_logfile<<"#### V_x=\t"<<m_curr_vel[0]<<"\tV_y=\t"<<m_curr_vel[1]<<"\tW_z=\t"<<m_curr_vel[2]<<endl;

            m_logfile<<"#### Velocity"<<m_curr_vel<<endl;

            vpDisplay::display(m_desired_image_rotated);
            vpDisplay::flush(m_desired_image_rotated);

            stringstream sstream_current_image;
            sstream_current_image<<m_logs_path<<"current_images/current_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')
                                <<m_vs_translation->get_iteration_number()<<".png";
            vpImageIo::write(m_current_image,sstream_current_image.str().c_str());

            m_sId->staticClean();

            //            if (m_curr_vel[0] < 5e-3)
            if (m_curr_vel[0] < 0.0)
            {
                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);

                this->m_combined_translation_and_rotation = false;
                this->m_second_rotation_mode = true;
                ros::spinOnce();
            }

            else
            {
                m_velocity.linear.x     =  m_curr_vel[0];
                m_velocity.angular.z    =  m_curr_vel[1];
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
            m_logfile<<"######### END TRANSLATION #########"<<endl;
            m_logfile<<endl;
            m_logfile<<endl;
            ros::spinOnce();

            */

            m_vs_translation->increment_iteration();


            // find best angle between the current image and and the desired image
            double step = 1.0;
//            if (m_vs_translation->get_iteration_number() == 0)
//            {
                m_theta_degrees_init = 0.0;
                m_theta_degrees_init = this->m_vs_translation->find_best_angle(m_desired_image, m_current_image , m_center,
                                                                               step, m_logfile);
                m_vs_translation->rotate_my_image(this->m_desired_image, m_center, m_theta_degrees_init,
                                                  this->m_desired_image_rotated);
                m_logfile<<"### VC Finding best angle between desired and current image\t"<<m_theta_degrees_init<<endl;


//            }
//            else
//            {
//                double range_min = m_theta_degrees_init - vpMath::sign(m_theta_degrees_init)*40;
//                double range_max = m_theta_degrees_init + vpMath::sign(m_theta_degrees_init)*40;
//                m_logfile<<"### range_min\t"<<range_min<<endl;
//                m_logfile<<"### range_max\t"<<range_max<<endl;
//                m_theta_degrees = 0.0;
//                m_theta_degrees = this->m_vs_translation->find_best_angle_within_range(m_desired_image, m_current_image , m_center,
//                                                                                       step, range_min,  range_max,  m_logfile);
//                m_vs_translation->rotate_my_image(this->m_desired_image, m_center, m_theta_degrees, this->m_desired_image_rotated);
//                m_logfile<<"### VC Finding best angle between desired and current image\t"<<m_theta_degrees<<endl;
//            }


            //rotate the desired image so that the current and desired image have the same orientation

            // display the image
            vpDisplay::display(m_desired_image_rotated);
            vpDisplay::flush(m_desired_image_rotated);

            vpImageTools::imageDifference(this->m_current_image, this->m_desired_image_rotated, this->m_difference_image_translation);
            vpDisplay::display(this->m_difference_image_translation);
            vpDisplay::flush(this->m_difference_image_translation);


            this->m_vec_current_images.push_back(m_current_image);
            this->m_vec_desired_images.push_back(m_desired_image_rotated);
            this->m_vec_diff_images.push_back(m_difference_image_translation);



            //
            // from the rotated image init visual servoing
            //


            // build features


            // init visual servo V_x
            CFeatureLuminanceOmni Si;
            Si.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Si.setCameraParameters(this->m_cam_param) ;
            Si.set_DOF(true,true, false, //Vx Vy Vz
                       false, false, false); //Wx Wy Wz
            Si.setInterpType(INTERPTYPE);


            //
            // build desired feature
            //
            CFeatureLuminanceOmni Sid;
            Sid.setInterpType(INTERPTYPE);
            Sid.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Sid.set_DOF(true, true, false,
                        false, false, false);
            Sid.buildFrom(m_desired_image_rotated);
            Sid.interaction(m_Lsd_translation) ;
            // Compute the Hessian H = L^TL
            m_Hsd_translation = m_Lsd_translation.AtA() ;
            // Compute the Hessian diagonal for the Levenberg-Marquardt
            unsigned int n = m_Lsd_translation.getCols() ;
            m_diagHsd_translation = vpMatrix(n,n) ;
            m_diagHsd_translation.eye(n);
            for(unsigned int ii = 0 ; ii < n ; ii++)
            {
                m_diagHsd_translation[ii][ii] = m_Hsd_translation[ii][ii];
            }

            //
            // Perform visual servoing iteration
            //
            Si.buildFrom(m_current_image);

            // Minimisation
            vpMatrix H;
            vpColVector v, e, error;


            vpRobust robust(0);
            robust.setThreshold(0.0);
            vpColVector w;
            vpColVector weighted_error;

            // compute current error
            Si.error(Sid,error);

            //        cout<<"#### error"<<error<<endl;
            w.resize(error.getRows());
            weighted_error.resize(error.getRows());
            w =1;
            robust.MEstimator(vpRobust::TUKEY, error, w);

            H = ((m_mu * (m_diagHsd_translation)) + (m_Hsd_translation)).inverseByLU();

            vpMatrix interaction_matrix((m_Lsd_translation).getRows(),(m_Lsd_translation).getCols());
            int nb_total_points = (m_Lsd_translation).getRows();

            for (int i=0; i<nb_total_points; i++)
            {
                weighted_error[i] = w[i]*error[i];
                interaction_matrix[i][0] = m_Lsd_translation[i][0]*w[i];
                interaction_matrix[i][1] = m_Lsd_translation[i][1]*w[i];
                //                interaction_matrix[i][2] = m_Lsd_translation[i][2]*w[i];

            }

            //	compute the control law
            e = H * interaction_matrix.t() * weighted_error;

            double normError  = weighted_error.sumSquare();

            if (m_vs_translation->get_iteration_number() > 0)
            {
                m_prev_vel = m_curr_vel;
                m_previous_error = m_current_error;
            }

            m_current_error = sqrt(normError)/m_height*m_width;

            m_curr_vel = - m_lambda*e;
            //            m_logerror<<m_vs_translation->get_iteration_number()<<"\t"<<m_curr_vel[0]<<"\t"<<m_curr_vel[1]<<"\t"<<m_curr_vel[2]<<endl;
            m_logerror<<m_vs_translation->get_iteration_number()<<"\t"<<m_curr_vel[0]<<"\t"<<m_curr_vel[1]<<endl;

            m_logfile<<"### iteration "<<m_vs_translation->get_iteration_number()<<endl;

            /// End
            /// Translation
            /// V_x
            /*

            // init visual servo V_y
            CFeatureLuminanceOmni Siy;
            Siy.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Siy.setCameraParameters(this->m_cam_param) ;
            Siy.set_DOF(false,true, false, //Vx Vy Vz
                        false, false, false); //Wx Wy Wz
            Siy.setInterpType(INTERPTYPE);

            //
            // build desired feature Sid_y
            //
            CFeatureLuminanceOmni Sid_y;
            Sid_y.setInterpType(INTERPTYPE);
            Sid_y.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Sid_y.set_DOF(false, true, false,
                          false, false, false);
            Sid_y.buildFrom(m_desired_image_rotated);
            Sid_y.interaction(m_Lsd_translation_y) ;

            // Compute the Hessian H = L^TL
            m_Hsd_translation_y = m_Lsd_translation_y.AtA() ;
            // Compute the Hessian diagonal for the Levenberg-Marquardt
            unsigned int n_y = m_Lsd_translation_y.getCols() ;
            m_diagHsd_translation_y = vpMatrix(n,n) ;
            m_diagHsd_translation_y.eye(n);
            for(unsigned int ii = 0 ; ii < n_y ; ii++)
            {
                m_diagHsd_translation_y[ii][ii] = m_Hsd_translation_y[ii][ii];
            }

            //
            // Perform visual servoing iteration
            //
            Siy.buildFrom(m_current_image);

            // Minimisation
            vpMatrix H_y;
            vpColVector v_y, e_y, error_y;


            vpRobust robust_y(0);
            robust_y.setThreshold(0.0);
            vpColVector w_y;
            vpColVector weighted_error_y;

            // compute current error
            Siy.error(Sid_y,error_y);

            //        cout<<"#### error"<<error<<endl;
            w_y.resize(error_y.getRows());
            weighted_error_y.resize(error_y.getRows());
            w_y =1;
            robust_y.MEstimator(vpRobust::TUKEY, error_y, w_y);

            H_y = ((m_mu * (m_diagHsd_translation_y)) + (m_Hsd_translation_y)).inverseByLU();

            vpMatrix interaction_matrix_y((m_Lsd_translation_y).getRows(),(m_Lsd_translation_y).getCols());
            int nb_total_points_y = (m_Lsd_translation_y).getRows();

            for (int i=0; i<nb_total_points_y; i++)
            {
                weighted_error_y[i] = w_y[i]*error_y[i];
                interaction_matrix_y[i][0] = m_Lsd_translation_y[i][0]*w_y[i];
//                interaction_matrix_y[i][1] = m_Lsd_translation_y[i][1]*w_y[i];
            }

            //	compute the control law
            e_y = H_y * interaction_matrix_y.t() * weighted_error_y;

            m_logfile<<"m_width\t"<<m_width<<"\tm_height=\t"<<m_height<<endl;
            double normError_y  = weighted_error_y.sumSquare();
            double current_error_y = sqrt(normError_y)/m_height*m_width;

            vpColVector curr_vel_y = - m_lambda*e_y;
            m_logerror<<m_vs_translation->get_iteration_number()<<"\t"<<m_curr_vel<<"\t"<<curr_vel_y<<endl;
            */


            double diff = fabs(m_previous_error - m_current_error);

            m_logfile<<"##### Diff\t"<<diff<<endl;
            double v_x = m_curr_vel[0];
            double lambda = 1.0;
            double omega = lambda*m_curr_vel[1];

            //            double omega_z = m_curr_vel[2];

            //            double omega = curr_vel_y[0];


            //            m_logfile<<"v_x=\t"<<v_x<<"\tomega=\t"<<omega<<"\tomega_z=\t"<<omega_z<<endl;

            m_logfile<<"### v_x=\t"<<v_x<<"\tomega=\t"<<omega<<endl;
            time_transl = vpTime::measureTimeMs() - time_transl;
            m_logfile<<"### time required =\t"<<time_transl<<endl;

//            if ( m_curr_vel[0] > 0.0)
            if ( m_curr_vel[0] > 0.0 && m_vs_translation->get_iteration_number() > 4)
            {
                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_combined_translation_and_rotation = false;
                this->m_second_rotation_mode = true;

                m_logfile<<endl;
                m_logfile<<endl;
                m_logfile<<"####### Entering second rotation meode"<<endl;

                Si.staticClean();
                Sid.staticClean();
                m_vs_translation->clean();

                ros::spinOnce();

            }
            else
            {
                m_logfile<<"#### previious velo is not available"<<endl;

                m_velocity.linear.x = v_x;
                m_velocity.angular.z = omega;
                m_ros_pub.publish(m_velocity);

                Si.staticClean();
                Sid.staticClean();
                m_vs_translation->clean();
                ros::spinOnce();
            }
            //            if (m_vs_translation->get_iteration_number() > 0)
            //            {
            //                m_logfile<<"#### previious velo is available"<<endl;
            //                if((vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1))

            //            if (v_x <0)
            //            {
            //                m_logfile<<"#### velcoity has changed its sign"<<endl;
            //                m_velocity.linear.x = 0.0;
            //                m_velocity.angular.z = 0.0;
            //                m_ros_pub.publish(m_velocity);
            //                this->m_combined_translation_and_rotation = false;
            //                this->m_second_rotation_mode = true;
            //                ros::spinOnce();
            //            }
            //            else
            //            {

            //                m_logfile<<"#### velo has not changed its sign yet"<<endl;
            //                m_velocity.linear.x = v_x;
            //                m_velocity.angular.z = omega;
            //                m_logfile<<"V_x\t"<<m_velocity.linear.x<<"\tW_z"<<m_velocity.angular.z<<endl;
            //                m_ros_pub.publish(m_velocity);
            //                ros::spinOnce();
            //            }
            //            }

            /*
            double time_transl = vpTime::measureTimeMs();

            m_vs_translation->increment_iteration();

            stringstream sstream_desired_image_rotated, sstream_current_image, sstream_diff_image;
            sstream_current_image<<m_logs_path<<"current_images/current_image_combined_trans_and_rot_mode_"<<m_vs_translation->get_iteration_number()<<".png";
            sstream_desired_image_rotated<<m_logs_path<<"desired_images/desired_image_rot_combined_trans_and_rot_mode_"<<m_vs_translation->get_iteration_number()<<".png";
            sstream_diff_image<<m_logs_path<<"diff_images/diff_image_rot_combined_trans_and_rot_mode_"<<m_vs_translation->get_iteration_number()<<".png";


            m_logfile<<"###################\t m_iter_combined_rot_and_trans\t"<<m_vs_translation->get_iteration_number()<<endl;




//            if (m_vs_translation->get_iteration_number() == 0)
//            {
                // find best angle between the current image and and the desired image
                int step = 2;
                m_theta_degrees = this->m_vs_translation->find_best_angle(m_desired_image, m_current_image , m_center,
                                                                      step, m_logfile);
                m_logfile<<"##VC Finding best angle between desired and current image\t"<<m_theta_degrees<<endl;
                // rotate the desired image so that the current and desired image have the same orientation
                m_vs_translation->rotate_my_image(this->m_desired_image, m_center, m_theta_degrees, this->m_desired_image_rotated);
//            }

            // display the image
            vpDisplay::display(m_desired_image_rotated);
            vpDisplay::flush(m_desired_image_rotated);

            vpImageTools::imageDifference(this->m_current_image, this->m_desired_image_rotated, this->m_difference_image_translation);
            vpDisplay::display(this->m_difference_image_translation);
            vpDisplay::flush(this->m_difference_image_translation);
            ;
            //
            // from the rotated image init visual servoing
            //

            vpImageIo::write(m_desired_image_rotated, sstream_desired_image_rotated.str().c_str());
            vpImageIo::write(m_difference_image_translation, sstream_diff_image.str().c_str());
            vpImageIo::write(m_current_image, sstream_current_image.str().c_str());


            //            vpColVector velocity;

            double omega = 0.0;
            // build features

            // init visual servo
            CFeatureLuminanceOmni Si;
            Si.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Si.setCameraParameters(this->m_cam_param) ;
            Si.set_DOF(true,true, false, //Vx Vy Vz
                       false, false, false); //Wx Wy Wz
            Si.setInterpType(INTERPTYPE);


            //
            // build desired feature
            //
            CFeatureLuminanceOmni Sid;
            Sid.setInterpType(INTERPTYPE);
            Sid.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_vs_translation->getRHO(), REPTYPE, GRAPCALCTYPE);
            Sid.set_DOF(true, true, false,
                        false, false, false);

            Sid.buildFrom(m_desired_image_rotated);
            Sid.interaction(m_Lsd) ;
            // Compute the Hessian H = L^TL
            m_Hsd = m_Lsd.AtA() ;
            // Compute the Hessian diagonal for the Levenberg-Marquardt
            unsigned int n = m_Lsd.getCols() ;
            m_diagHsd = vpMatrix(n,n) ;
            m_diagHsd.eye(n);
            for(unsigned int ii = 0 ; ii < n ; ii++)
            {
                m_diagHsd[ii][ii] = m_Hsd[ii][ii];
            }

            //
            // Perform visual servoing iteration
            //
            Si.buildFrom(m_current_image);

            // Minimisation
            vpMatrix H;
            vpColVector v, e, error;


            vpRobust robust(0);
            robust.setThreshold(0.0);
            vpColVector w;
            vpColVector weighted_error;

            // compute current error
            Si.error(Sid,error);

            //        cout<<"#### error"<<error<<endl;
            w.resize(error.getRows());
            weighted_error.resize(error.getRows());
            w =1;
            robust.MEstimator(vpRobust::TUKEY, error, w);

            H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();

            vpMatrix interaction_matrix ((m_Lsd).getRows(),(m_Lsd).getCols());
            int nb_total_points = (m_Lsd).getRows();

            for (int i=0; i<nb_total_points; i++)
            {
                weighted_error[i] = w[i]*error[i];
                interaction_matrix[i][0] = m_Lsd[i][0]*w[i];
                interaction_matrix[i][1] = m_Lsd[i][1]*w[i];
            }


            //	compute the control law
            e = H * interaction_matrix.t() * weighted_error;

            if (m_vs_translation->get_iteration_number() > 0)
            {
                m_prev_vel = m_curr_vel;
                m_previous_error = m_current_error;


            }

            m_logfile<<"m_width\t"<<m_width<<"\tm_height=\t"<<m_height<<endl;
            double normError  = weighted_error.sumSquare();
            m_current_error = sqrt(normError)/m_height*m_width;


            m_logfile<<"m_prev_vel\t"<<m_prev_vel<<"\n m_curr_ve\t"<<m_curr_vel<<endl;
            m_logfile<<"m_previous_error"<<m_previous_error<<"\n m_current_error\t"<<m_current_error<<endl;

            m_logerror<<m_vs_translation->get_iteration_number()<<"\t"<<m_current_error<<endl;
            m_curr_vel = - m_lambda*e;



            double new_theta = atan2(m_curr_vel[1], m_curr_vel[0]);

            if (new_theta > M_PI/2.0 )
            {
                new_theta -= M_PI;
            }

            if (new_theta < -M_PI/2.0 )
            {
                new_theta += M_PI ;
            }

            double new_theta_degrees = vpMath::deg(new_theta);
            m_logfile<<"###VS new theta\t"<<new_theta_degrees<<endl;
            m_log_angles<<m_vs_translation->get_iteration_number()<<"\t"<<new_theta_degrees<<endl;

            double lambda = m_lambda_rot;
            //            omega = 10.0*fabs(velocity[0])*new_theta;
            omega = fabs(m_curr_vel[0])*new_theta;

            if((fabs(omega) > 0.1) ||  (fabs(m_curr_vel[0]) > 0.1))
            {
                m_lambda /= 2;
                m_mu /= 2;
                m_curr_vel = - m_lambda*e;
                omega = fabs(m_curr_vel[0])*new_theta;
            }

            m_logfile<<"velocity\t"<<m_curr_vel<<endl;
            m_logfile<<"omega\t"<<omega<<endl;
            double diff = fabs(m_current_error - m_previous_error);

            m_logfile<<"### Diff\t"<<diff<<endl;



            m_logfile<<"m_lambda=\t"<<m_lambda<<"\tm_mu="<<m_mu<<endl;

            //                            if(vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1 && m_vs_translation->get_iteration_number() > 10)

//            if (diff < m_threshold_error)
//            {
//                if (m_prev_vel.data)
//                {
//                    if(vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1)
//                    {

            if (diff < m_threshold_error)
            {

                m_logfile<<"#### error is smaller than threshold"<<endl;
                if (m_prev_vel.data)
                {
                    m_logfile<<"#### previious velo is available"<<endl;

                    if((vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1 && m_vs_translation->get_iteration_number() > 10) || fabs(m_curr_vel[0]) <1e-3)
                    {

                        m_logfile<<"#### velcoity has changed its sign"<<endl;
                        m_velocity.linear.x = 0.0;
                        m_velocity.angular.z = 0.0;
                        m_ros_pub.publish(m_velocity);
                        this->m_combined_translation_and_rotation = false;
                        this->m_second_rotation_mode = true;
                        ros::spinOnce();
                    }
                    else
                    {

                        m_logfile<<"#### velo has not changed its sign yet"<<endl;
                        m_velocity.linear.x = m_curr_vel[0];
                        m_velocity.angular.z = omega;
                        m_logfile<<"V_x\t"<<m_velocity.linear.x<<"\tW_z"<<m_velocity.angular.z<<endl;
                        m_ros_pub.publish(m_velocity);
                        ros::spinOnce();
                    }
                }
                else
                {
                    m_logfile<<"#### previious velo is not available"<<endl;

                    m_velocity.linear.x = m_curr_vel[0];
                    m_velocity.angular.z = omega;
                    m_logfile<<"V_x\t"<<m_velocity.linear.x<<"\tW_z"<<m_velocity.angular.z<<endl;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }

            }
            else
            {

                m_logfile<<"#### error is bigger than threshold"<<endl;

                m_velocity.linear.x = m_curr_vel[0];
                m_velocity.angular.z = omega;
                m_logfile<<"V_x\t"<<m_velocity.linear.x<<"\tW_z"<<m_velocity.angular.z<<endl;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }


*/
            /*

            //            m_vs_translation->init_visual_servo(this->m_sI_translation, true, true, false, false, false, false);
            //            m_vs_translation->build_desired_feature(m_desired_image_rotated, true, true, false, false, false, false);

            //            m_logfile<<"######### BEGIN ROTATION #########"<<endl;

            //            if (m_vs_translation->get_iteration_number() > 0)
            //            {
            //                m_prev_vel = m_curr_vel;
            //                m_logfile<<"m_prev_vel\t"<<m_prev_vel<<"\n m_curr_ve\t"<<m_curr_vel<<endl;
            //            }

            //            this->m_vs_translation->perform_a_VS_iteration_with_MEstimator(this->m_sI_translation, m_current_image,
            //                                                                        m_curr_vel, m_logfile);



            //            m_logfile<<"### Vel_x\t"<<m_curr_vel[0]<<"\tVel_y\t"<<m_curr_vel[1]<<endl;
            //            double new_theta = atan2(m_curr_vel[1], m_curr_vel[0]);

            //            if (new_theta > M_PI/2.0 )
            //            {
            //                new_theta -= M_PI;
            //            }

            //            if (new_theta < -M_PI/2.0 )
            //            {
            //                new_theta += M_PI ;
            //            }

            //            double new_theta_degrees = vpMath::deg(new_theta);
            //            m_logfile<<"### new theta\t"<<new_theta_degrees<<endl;
            //            m_log_angles<<m_vs_translation->get_iteration_number()<<"\t"<<new_theta_degrees<<endl;

            //            double lambda = m_lambda_rot;
            //            //            omega = 10.0*fabs(velocity[0])*new_theta;
            //            omega = fabs(m_curr_vel[0])*new_theta;

            //            m_logfile<<"velocity\t"<<m_curr_vel<<endl;
            //            m_logfile<<"omega\t"<<omega<<endl;

            //            double current_error = m_vs_translation->getCurrentError();
            //            double prev_error  = m_vs_translation->getPreviousError();

            //            double diff = fabs(current_error - prev_error);
            //            m_logfile <<"#### DIFF= "<<diff<<endl;

            //            time_transl = vpTime::measureTimeMs() - time_transl;
            //            m_logfile<<"### Required Time for processing \t"<<time_transl<<endl;


            //            m_logerror<<m_vs_translation->get_iteration_number()<<"\t"<<current_error<<endl;

            //            m_vs_translation->clean();




            //            m_logfile<<"m_error_threshold"<<m_threshold_error<<endl;

            //            if (diff < m_threshold_error)
            //            {
            //                m_logfile<<"## diff is less than threshold"<<endl;

            //                //                if(vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1 && m_vs_translation->get_iteration_number() > 10)
            //                if (m_prev_vel.data)
            //                {
            //                    if(vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1)
            //                    {
            //                        m_logfile<<"## velcoity has changed its sign"<<endl;
            //                        m_velocity.linear.x = 0.0;
            //                        m_velocity.angular.z = 0.0;
            //                        m_ros_pub.publish(m_velocity);
            //                        this->m_combined_translation_and_rotation = false;
            //                        this->m_second_rotation_mode = true;
            //                        ros::spinOnce();
            //                    }
            //                }

            //            }
            //            else
            //            {
            //                //                if (fabs(m_curr_vel[0]) < 0.1)
            //                //                {
            //                //                    m_curr_vel = vpMath::sign(m_curr_vel[0])*0.1;
            //                //                }

            //                //                if (fabs(omega) < 0.1)
            //                //                {
            //                //                    omega = vpMath::sign(omega)*0.1;
            //                //                }
            //                m_velocity.linear.x = 5*m_curr_vel[0];
            //                m_velocity.angular.z = 2.5*omega;
            //                m_ros_pub.publish(m_velocity);
            //                ros::spinOnce();
            //            }
*/
            /*
            if (m_vs_translation->getCurrentError()< 0.01)
            {
                if (m_prev_vel.data)
                {
                    m_logfile<<"sign curr vel"<<vpMath::sign(m_curr_vel[0])<<"\t sign prev vel"<<vpMath::sign(m_prev_vel[0])<<endl;

                    // when the current velocity changes its sign then stop the robot
                    if(vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0]) == -1 && m_vs_translation->get_iteration_number() > 30)
                    {
                        m_logfile<<"signs have been changed"<<endl;
                        m_counter++;
                    }
                }

            }

            if (m_counter == 5)
            {
                m_logfile<<"m_counter\t"<< m_counter <<endl;
                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_combined_translation_and_rotation = false;
                this->m_second_rotation_mode = true;
                ros::spinOnce();

            }
            */

        }

        else if (m_second_rotation_mode)
        {

            m_iter++;

            //
            // write date
            //
            if (m_iter == 0)
            {
                this->write_date();
            }

            stringstream sstream_filename;
            sstream_filename<<m_logs_path<<"current_images/current_image_second_rotation_mode_"<<std::setw(4) << std::setfill('0')<<m_iter<<".png";
            vpImageIo::write(m_current_image, sstream_filename.str().c_str());




            //
            //  display diff image
            //
            vpImageTools::imageDifference(this->m_current_image, this->m_desired_image, this->m_difference_image_rotation) ;
            vpDisplay::display(m_difference_image_rotation);
            vpDisplay::flush(m_difference_image_rotation);
            stringstream sstream_diff;
            sstream_diff<<m_logs_path<<"diff_images/diff_image_second_rotation_mode_"<<std::setw(4) << std::setfill('0')<<m_iter<<".png";
            vpImageIo::write(m_difference_image_rotation, sstream_diff.str().c_str());
            //            double required_angle_degrees = m_required_angle*M_PI/180.0;
            int step  = 2;

            m_logfile<<"### entering second rotation mode"<<endl;
            double theta_prime_degrees = 0.0;

            theta_prime_degrees = this->m_visual_servo_tools.find_best_angle(this->m_desired_image, m_current_image,
                                                                             m_center, step, m_logfile);


            //            theta_prime_degrees = this->m_visual_servo_tools.find_best_angle_within_range(this->m_desired_image, m_current_image,
            //                                                                                          m_center, step,0, m_required_angle +20, m_logfile);

            double theta_prime_radians = theta_prime_degrees * M_PI/180.0;
            double lambda_rot = 0.4;
            double omega;
            //            double new_lambda = -lambda_rot;
            //            if (m_required_angle < 0)
            //            {
            //                omega = -new_lambda*(theta_prime_radians);
            //            }
            //            else
            //            {
            omega = -lambda_rot*(theta_prime_radians);
            //            }

            m_velocity.linear.x = 0.0;
            //            m_logfile<<"m_nb_pixels"<<m_width*m_height*3/4<<endl;
            m_logfile<<"## Radians : required\t"<<m_required_angle<<"\tvisual compass:\t"<<theta_prime_radians<<endl;
            m_logfile<<"## Degrees : required\t"<<m_required_angle * 180.0/M_PI<<"\tvisual_compass:\t"<<theta_prime_degrees<<endl;
            m_logfile<<"velocity\t"<<v<<endl;
            m_logfile<<"angular velocity"<<omega<<endl;

            if (fabs(omega)< 2e-3)
                //            if (fabs(theta_prime_degrees ) <= 1.0)
            {
                m_logfile<<"omega is almost zero"<<endl;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_second_rotation_mode = false;
                ros::spinOnce();
            }

            else
            {
                m_logfile<<"######## in else"<<endl;
                m_velocity.angular.z = omega + vpMath::sign(omega)*0.05;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
        }

    }

    void write_date()
    {

        int size = this->m_vec_current_images.size();
        for(int i= 0; i<size; i++)
        {
            stringstream sstream_desired_image_rotated, sstream_current_image, sstream_diff_image;
            sstream_current_image<<m_logs_path<<"current_images/current_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_desired_image_rotated<<m_logs_path<<"desired_images/desired_image_rot_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_diff_image<<m_logs_path<<"diff_images/diff_image_rot_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";

            vpImageIo::write(this->m_vec_current_images[i], sstream_current_image.str().c_str());
            vpImageIo::write(this->m_vec_desired_images[i], sstream_desired_image_rotated.str().c_str());
            vpImageIo::write(this->m_vec_diff_images[i], sstream_diff_image.str().c_str());
        }

    }

private:


    double m_sad;
    int m_nb_cycles;
    int m_nb_black_pixels;
    int m_index_desired_image_fwd;
    int m_index_desired_image_bwd;
    int m_counter;

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    //    ros::Publisher m_ros_pub_2;
    //    ros::Subscriber m_sonar_sub;
    ros::Subscriber m_ros_sub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_current_image_rerotated;

    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_initial_image;

    vpImage<unsigned char> m_temp_desired_image;
    vpImage<unsigned char> m_desired_image_rotated;

    vpImage<unsigned char> m_desired_image_rotated_rerotated;
    vpImage<unsigned char> m_new_desired_image_rotated;
    vpImage<unsigned char> m_desired_image_second_rotation;

    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_difference_image_translation;
    vpImage<unsigned char> m_difference_image_rotation;

    vpImage<unsigned char> m_mask_image;

    vpImage<unsigned char> m_temp_curr;
    vpImage<unsigned char> m_temp_des;


    ofstream m_logfile;
    ofstream m_logerror;
    ofstream m_log_angles;
    ofstream m_log_omega;

    string m_logs_path;
    string m_data_path;
    CFeatureLuminanceOmni m_sI_first_rotation ;
    CFeatureLuminanceOmni m_sI_translation ;
    CFeatureLuminanceOmni m_sI_second_rotation ;

    CFeatureLuminanceOmni *m_sId_translation;
    CFeatureLuminanceOmni *m_sId_first_rotation;
    CFeatureLuminanceOmni *m_sId_second_rotation;
    CFeatureLuminanceOmni *m_sId_theta;

    vpMatrix m_Lsd_first_rotation, m_Lsd_second_rotation, m_Lsd_translation, m_Lsd_translation_y;
    vpMatrix m_Hsd_first_rotation, m_Hsd_second_rotation, m_Hsd_translation, m_Hsd_translation_y;
    vpMatrix m_diagHsd_first_rotation, m_diagHsd_second_rotation, m_diagHsd_translation, m_diagHsd_translation_y;

    int m_nb_croissance;

    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;

    //    int m_iter_rotation;
    int m_iter_combined_rot_and_trans;

    int m_iter_g;
    geometry_msgs::Twist m_velocity;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_current_image_rerotated;

    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_desired_image_rot;
    vpDisplayX m_display_desired_image_rot_rerot;
    vpDisplayX m_display_error;
    vpDisplayX m_display_error_translation;
    vpDisplayX m_display_error_rotation;


    vpDisplayX m_display_temp_curr;
    vpDisplayX m_display_temp_des;

    double m_current_error;
    double m_previous_error;
    double m_pred_error;
    double m_initial_error;
    double m_mu;
    double m_lambda;
    double m_lambda_rot;
    int m_first_forward;
    int m_first_backward;
    int m_nb_tours;
    double m_threshold_error;
    double m_diff_error;
    double m_mean_diff_error;
    double m_required_angle;
    double m_required_angle_degrees;
    double m_theta_degrees;
    double m_theta_degrees_init;

    std::vector<double> m_angles;
    double m_new_required_angle;
    double m_new_required_angle_degrees;
    double m_mean_required_angles;

    //    double m_odometry_angle;

    //    double m_lambda_rot, m_mu_rot, m_current_error_rot, m_pred_error_rot;
    //    double m_lambda_tran, m_mu_tran, m_current_error_tran, m_pred_error_tran;

    bool m_first_rotation_mode;
    bool m_second_rotation_mode;
    bool m_combined_translation_and_rotation;

    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_center[2];
    double m_center_image[2];
    VisualServoTools m_visual_servo_tools;
    VisualServoTools *m_vs_translation;
    VisualServoTools *m_vs_first_rot;

    vpColVector m_prev_vel;
    vpColVector m_curr_vel;

    CFeatureLuminanceOmni *m_sId;
    vpMatrix m_Lsd, m_Hsd, m_diagHsd;
    CCameraOmniParameters m_cam_param;
    vpColVector m_curr_vel_first_rot;
    vpColVector m_prev_vel_first_rot;

    std::vector<vpImage<unsigned char> > m_vec_current_images;
    std::vector<vpImage<unsigned char> > m_vec_diff_images;
    std::vector<vpImage<unsigned char> > m_vec_desired_images;

};

