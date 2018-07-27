#include "VisualPathFollowingWithPureRotation.h"


VisualPathFollowingWithPureRotation::VisualPathFollowingWithPureRotation()
    : m_it(m_nh)
{

    m_move_forward = true;
    m_move_backward = false;

    m_theta_degrees = 0.0;
    m_nb_tours = 0;

    m_width     = m_combined_rotation_and_translation.getWidth();
    m_height    = m_combined_rotation_and_translation.getHeight();

    m_nb_croissance = 0;

    m_index_desired_image_fwd  = 0;
    m_index_desired_image_fwd  = 0;
    string cameraTopic, robotTopic, visualServoMovements;

    m_nh.param("cameraTopic", cameraTopic, string(""));
    m_nh.param("robotTopic", robotTopic, string(""));

    m_nh.param("visualServoMovements", visualServoMovements, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("data", m_data_path,string(""));
    m_nh.param("error_threshold",m_threshold_error, double(0.0));

    m_nh.param("alpha", m_alpha, double(0.0));
    m_nh.param("lambda", m_lambda, double(0.0));
    m_nh.param("mu", m_mu, double(0.0));

    m_nh.getParam("cameraTopic", cameraTopic);
    m_nh.getParam("visualServoMovements", visualServoMovements);
    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("data", m_data_path);
    m_nh.getParam("error_threshold",m_threshold_error);
    m_nh.getParam("robotTopic", robotTopic);
    m_nh.getParam("alpha", m_alpha);

    m_px  = m_combined_rotation_and_translation.getPx();
    m_py  = m_combined_rotation_and_translation.getPy();
    m_u0  = m_combined_rotation_and_translation.getU0();
    m_v0  = m_combined_rotation_and_translation.getV0();
    m_xi  = m_combined_rotation_and_translation.getXi();
    m_rho = m_combined_rotation_and_translation.getRHO();

    m_center[0] = m_u0;
    m_center[1] = m_v0;

    m_center_image[0] = m_height/2;
    m_center_image[1] = m_width/2;

    m_iter = -1;
    stringstream str;
    str<<m_logs_path<<"logfile_visual_servo_with_pure_rotation.txt";
    m_logfile.open(str.str().c_str());

    m_logfile<<"m_lambda\t"<<m_lambda<<"\tm_mu\t"<<m_mu<<"\tm_alpha"<<m_alpha<<endl;
    stringstream str_error;
    str_error<<m_logs_path<<"error.mat";
    m_logerror.open(str_error.str().c_str());

    m_current_image.init(m_height, m_width);
    m_desired_image.init(m_height, m_width);
    m_difference_image.init(m_height, m_width);
    m_prev_desired_image.init(m_height, m_width);

//        m_desired_image_rotated.init(m_height, m_width);

    m_display_current_image.init(m_current_image, 0, 0, "Current image");
    m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
//        m_display_rotated_desired_image.init(this->m_desired_image_rotated, 2*m_width+10, 2*m_height+10, "Rotated desired image");
    m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");
    m_display_prev.init(m_prev_desired_image, 2*m_width+10, 2*m_height+10, "Previous desired image");

    initVisualPathFollowingWithPureRotation();

    m_image_sub = m_it.subscribe(cameraTopic, 1, &VisualPathFollowingWithPureRotation::imageCallback, this);
    m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1000);

}

VisualPathFollowingWithPureRotation::~VisualPathFollowingWithPureRotation()
{
}

void
VisualPathFollowingWithPureRotation::initVisualPathFollowingWithPureRotation()
{

    m_logfile<<"#### Threshold error"<<m_threshold_error<<endl;

    //
    // Load image mask
    //
    stringstream  sstr_mask;
    sstr_mask<<m_data_path<<"mask/mask.png";
    string filename_mask = sstr_mask.str();




    vpImage<unsigned char> tmp_mask(m_width, m_height);
    m_vs_forward.read_mask(filename_mask, tmp_mask);

    tmp_mask.halfSizeImage(m_mask_image);

    m_vs_forward.init_visual_servo(m_sI, true, false, false, false, false, true);

    m_vs_forward.init_params_without_MEstimator();
    m_vs_forward.initialize();

    // Load desired images
    stringstream str;
    str<<m_logs_path<<"desired_images/";
    string filename_prefix = "desired_image";
    std::vector<string> filenames_desired_images;

    m_vs_forward.load_directory_and_sort_files(str.str().c_str(), filename_prefix.c_str(), filenames_desired_images);

    this->m_vec_desired_images = std::vector<vpImage<unsigned char> > (filenames_desired_images.size());

    //
    // read desired images
    //
    for (unsigned int i = 0;  i< filenames_desired_images.size(); i++)
    {
        stringstream  sstr_des;
        string filename;
        m_vec_desired_images[i].init(this->m_height, m_width);
        sstr_des <<m_logs_path<<"desired_images/"<<filenames_desired_images[i];
        filename =sstr_des.str();
        vpImageIo::read(m_vec_desired_images[i],filename);

        CFeatureLuminanceOmni *sId = new CFeatureLuminanceOmni();

        vpMatrix Lsd, Hsd, diagHsd;

        m_vs_forward.build_desired_feature(this->m_vec_desired_images[i],
                                                   true, false, false,
                                                   false, false, true,
                                                   sId, m_Lsd, m_Hsd, m_diagHsd);
//            m_logfile<<"building desired feature "<<i<<endl;
        m_vec_sId.push_back(sId);
        m_vec_Lsd.push_back(m_Lsd);
        m_vec_Hsd.push_back(m_Hsd);
        m_vec_diagHsd.push_back(m_diagHsd);
    }
}


void
VisualPathFollowingWithPureRotation::reverse_vectors()
{
    m_logfile<<"##### reverting vectors"<<endl;
    std::reverse(m_vec_desired_images.begin(), m_vec_desired_images.end());
    std::reverse(m_vec_sId.begin(), m_vec_sId.end());
    std::reverse(m_vec_Lsd.begin(), m_vec_Lsd.end());
    std::reverse(m_vec_Hsd.begin(), m_vec_Hsd.end());
    std::reverse(m_vec_diagHsd.begin(), m_vec_diagHsd.end());
}

void
VisualPathFollowingWithPureRotation::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    m_iter++;
    //
    // acquire image
    //
    vpImage<unsigned char> curr_im_temp(m_height, m_width);
    curr_im_temp = visp_bridge::toVispImage(*image);
    curr_im_temp.halfSizeImage(m_current_image);

    m_combined_rotation_and_translation.multiply_image_by_mask(m_current_image, m_mask_image);

    vpDisplay::display(m_current_image);
    vpDisplay::flush(m_current_image);

    vpColVector e ;
    vpColVector v ; // camera velocity send to the robot

    if (m_move_forward)
    {
        m_logfile<<"################# MOVING BACKWARD ############################"<<endl;
        this->m_vs_forward.increment_iteration();
        m_logfile<<"########################### iteration"<<m_vs_forward.get_iteration_number()<<endl;
        m_vs_forward.multiply_image_by_mask(m_current_image, m_mask_image);

        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

        vpColVector e ;
        vpColVector v ; // camera velocity send to the robot

        stringstream str_curr, str_diff;
        m_logfile<<"size desired images vector"<<m_vec_desired_images.size() <<endl;

        if (m_index_desired_image_fwd < this->m_vec_desired_images.size() )
        {
            this->m_desired_image = this->m_vec_desired_images[this->m_index_desired_image_fwd];
            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);

            m_logfile<<"################################ Moving forward loading desired image n "<<m_index_desired_image_fwd<<endl;

            m_logfile<<"m_current_image"<<m_current_image.getCols()<<endl;
            m_logfile<<"m_desired_image"<<m_desired_image.getCols()<<endl;
            m_logfile<<"m_difference_image"<<m_difference_image.getCols()<<endl;

            vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
            vpDisplay::display(m_difference_image);
            vpDisplay::flush(m_difference_image);
            str_curr<<m_logs_path<<"current_images/current_image_"<<this->m_index_desired_image_fwd<<"_"<<m_vs_forward.get_iteration_number()<<".png";
            vpImageIo::write(m_current_image,str_curr.str());

            str_diff<<m_logs_path<<"diff_images/diff_image_"<<this->m_index_desired_image_fwd<<"_"<<m_vs_forward.get_iteration_number()<<".png";
            vpImageIo::write(m_difference_image, str_diff.str());

            double t = vpTime::measureTimeMs();
            m_logfile<<"m_index_desired_image_fwd\t"<<m_index_desired_image_fwd<<"\tm_diaghsd\t"<<m_vec_diagHsd[m_index_desired_image_fwd]<<endl;

            m_logfile<<"### Current desired image\t"<<m_index_desired_image_fwd<<endl;
            double angle_degrees;
            double step = 2.0;
            double angle_radians;
            double lambda = 0.25;
            double omega;

            // get the previous desired image when it is available
            if ((m_index_desired_image_fwd >= 1)  && (m_index_desired_image_fwd < this->m_vec_desired_images.size()-1))
            {
                this->m_prev_desired_image = this->m_vec_desired_images[this->m_index_desired_image_fwd-1];
                vpDisplay::display(m_prev_desired_image);
                vpDisplay::flush(m_prev_desired_image);

                //
                // compute angle between the desired and previous image
                //
                angle_degrees = m_vs_forward.find_best_angle(m_prev_desired_image, m_desired_image, m_center, step);
                angle_radians = angle_degrees*M_PI/180.0;

                m_logfile<<"angle degrees betwwen desired and previous"<<angle_degrees<<endl;

                if ((fabs(angle_degrees) > 5) && m_flag )
                {
                    m_logfile<<"### pure rotation mode triggered"<<endl;
                    m_flag = false;
                    m_rotation_mode = true;
                    m_translation_mode = false;
                }
            }

            if (m_rotation_mode)
            {
                m_logfile<<"entering pure rotation mode"<<endl;
                // rotation mode
                double theta_degrees = m_vs_forward.find_best_angle(m_desired_image, m_current_image , m_center, step);

                m_logfile<<"theta degrees\t"<<theta_degrees<<endl;
                double theta_radians = theta_degrees*M_PI/180.0;
                omega = -lambda*(theta_radians);

                m_logfile<<"## omega\t"<<omega<<endl;
                //                        if (fabs(theta_degrees - angle_degrees) < 2.)
                if (fabs(omega) < 2e-3)
                {
                    m_logfile<<"omega is almost zero"<<endl;
                    m_velocity.linear.x = 0.0;
                    m_velocity.angular.z = 0.0;

                    m_rotation_mode = false;
                    m_translation_mode = true;

                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }
                else
                {
                    m_velocity.linear.x = 0.0;
                    m_velocity.angular.z = omega+vpMath::sign(omega)*0.05;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }

            }

            if (m_translation_mode && (m_index_desired_image_fwd < this->m_vec_desired_images.size()))
            {
                m_logfile<<"#### entering translation mode"<<endl;
                m_logfile<<"m_index_desired_image_fwd\t"<<m_index_desired_image_fwd<<endl;
                m_vs_forward.perform_a_VS_iteration_without_MEstimator(m_sI,this->m_vec_sId[m_index_desired_image_fwd],
                                                                     m_current_image,
                                                                     v, this->m_vec_Lsd[m_index_desired_image_fwd],
                                                                     m_vec_Hsd[m_index_desired_image_fwd], m_vec_diagHsd[m_index_desired_image_fwd], m_logfile);



                t = vpTime::measureTimeMs() - t;
                m_logfile << "Time per frame "<<t<< std::endl;
                m_logfile << "### Velocity "<<v.t()<< std::endl;
                m_logfile << "### Iteration "<<m_vs_forward.get_iteration_number()<< std::endl;
                m_logfile << "### m_threshold_error "<<m_threshold_error << std::endl;



                if ( (this->m_vs_forward.getCurrentError() < m_threshold_error)  || (m_vs_forward.get_iteration_number() > 40) )
                {
                    if ((v[0] < 0.0))
                    {
                        m_index_desired_image_fwd++;
                        m_flag = true;
                        m_vs_forward.initialize();
                        m_vs_forward.init_params_without_MEstimator(m_logfile);
                    }

                    else
                    {
                        if (v[0] < 0.1 && v[0]>0.0)
                        {
                            v[0] = 0.1;
                        }
                        m_velocity.linear.x = v[0];
                        m_velocity.angular.z = v[1];
                        m_ros_pub.publish(m_velocity);
                        ros::spinOnce();
                    }
                }
                else if (v[0]< 0.0 && (m_index_desired_image_fwd != (this->m_vec_desired_images.size() - 1)))
                {
                    m_velocity.linear.x = 0.0;
                    m_velocity.angular.z = 0.0;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }
                else
                {
                    if (v[0] < 0.1 && v[0] > 0.0 )
                    {
                        v[0] = 0.1;
                    }
                    m_logfile<<" in if else"<<endl;
                    m_velocity.linear.x = v[0];
                    m_velocity.angular.z = v[1];
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }

            }
            else
            {
                m_logfile<<"neither rotation nor translation"<<endl;
            }
        }
        else
        {
            m_logfile<<"### end of visual path sending 0 velocity to robot"<<endl;
            m_velocity.linear.x = 0.0;
            m_velocity.angular.z = 0.0;
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
            ros::shutdown();
        }
    }
}
