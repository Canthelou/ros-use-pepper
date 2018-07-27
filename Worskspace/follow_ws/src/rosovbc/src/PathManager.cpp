

#include "PathManager.h"
#include <nav_msgs/Odometry.h>

PathManager::PathManager() : m_it(m_nh)
{
    m_curr_vel_x = 9999.9;
    m_curr_vel_y = 9999.9;

    m_save_desired_features = true;
    m_move_forward = true;
    m_move_backward = false;
    m_move_backward_visual_compass = false;
    m_nb_backward = 0;
    m_index_desired_image_fwd = 0;
    m_index_desired_image_bwd = 0;
    m_rotation_done = false;
    string cameraTopic;
    string robotTopic;

    m_acquisition = true;

    m_nh.param("cameraTopic", cameraTopic, string(""));
    m_nh.param("robotTopic", robotTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("data", m_data_path, string(""));

    m_nh.param("error_threshold", m_threshold_error, double(0.0));
    m_nh.param("ratio_vel_threshold", m_ratio_threshold, double(0.0));
    m_nh.param("diff_vel_threshold", m_diff_threshold, double(0.0));
    m_nh.param("max_iter_threshold", m_max_iters, int(0));


    m_nh.param("axis_linear", m_linear, m_linear);
    m_nh.param("axis_angular", m_angular, m_angular);
    m_nh.param("scale_angular", m_a_scale, m_a_scale);
    m_nh.param("scale_linear", m_l_scale, m_l_scale);

    m_nh.getParam("cameraTopic", cameraTopic);
    m_nh.getParam("robotTopic", robotTopic);
    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("data", m_data_path);
    m_nh.getParam("error_threshold",m_threshold_error);

    m_nh.getParam("ratio_vel_threshold", m_ratio_threshold);
    m_nh.getParam("diff_vel_threshold", m_diff_threshold);
    m_nh.getParam("max_iter_threshold", m_max_iters);


    ROS_INFO("####################### m_threshold_error%f\n ", m_threshold_error);

    m_nb_velocity_null = 0;
    stringstream str;
    str<<m_logs_path<<"logfile_path_manager.txt";
    m_logfile.open(str.str().c_str());
    m_logfile<<m_threshold_error<<"\t"<<m_ratio_threshold<<"\t"<<m_diff_threshold<<"\t"<<m_max_iters<<endl;

    stringstream str_err;
    str_err<<m_logs_path<<"error_path_backward.mat";
    m_logerror.open(str_err.str().c_str());


    stringstream str_poly;
    str_poly<<m_logs_path<<"poly.mat";
    m_log_poly.open(str_poly.str().c_str());


    stringstream str_vel;
    str_vel<<m_logs_path<<"velocity.mat";
    m_log_vel.open(str_vel.str().c_str());

    stringstream str_odom;
    str_odom<<m_logs_path<<"odom.mat";
    m_log_odom.open(str_odom.str().c_str());


    m_width = m_vs_forward.getWidth();
    m_height = m_vs_forward.getHeight();
    m_iter = -1;

    m_desired_image.init(m_height,m_width);
    m_current_image.init(m_height,m_width);
    m_difference_image.init(m_height,m_width);
    m_prev_desired_image.init(m_height, m_width);

    m_display_current_image.init(m_current_image, 0, 0, "Current image");
    m_display_desired_image.init(m_desired_image, m_width + m_width/4, 0, "Desired image");
    m_display_error.init(m_difference_image, 2*m_width + m_width/4, 0, "Difference");
    m_display_previous_desired_image.init(m_prev_desired_image, m_width+m_width/4, m_height+m_height/4, "Previous desired image");

    m_px                                = m_vs_forward.getPx();
    m_py                                = m_vs_forward.getPy();
    m_u0                                = m_vs_forward.getU0();
    m_v0                                = m_vs_forward.getV0();
    m_xi                                = m_vs_forward.getXi();
    m_rho                               = m_vs_forward.getRHO();
    m_center_rotation[0]                = m_u0;
    m_center_rotation[1]                = m_v0;

    m_center_image[0]                   = m_width;
    m_center_image[1]                   = m_height;

    initPathManager();

    m_vs_forward.initialize();
    this->m_vs_forward.init_params_without_MEstimator(m_logfile);
    m_vs_backward.init_params_without_MEstimator();
    m_vs_backward.initialize();

    m_image_sub = m_it.subscribe(cameraTopic, 1, &PathManager::imageCallback, this);
    m_nb_decroissace_error = 0;
    m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1000);
    m_robot_sub = m_nh.subscribe<geometry_msgs::Twist>(robotTopic, 10, &PathManager::robotVelocityCallback, this);

    m_odom_sub = m_nh.subscribe<nav_msgs::Odometry>("RosAria/pose", 1, &PathManager::robotOdometryCallback, this);

    m_timer = m_nh.createTimer(ros::Duration(10.0), &PathManager::callbackTimer, this);
    m_ros_pub_im_filename = m_nh.advertise<std_msgs::String >("/image_filename", 1000);
    m_flag  = true;
    m_required_angle = 0;
    m_translation_mode = false;
    m_rotation_mode = true;
}


PathManager::~PathManager()
{
    m_logfile.close();

    for (int i=0; i<m_vec_sId.size();i++)
    {
        delete m_vec_sId[i];
    }
}


void PathManager::initPathManager()
{
    //
    // Load image mask
    //
    stringstream  sstr_mask;
    sstr_mask<<m_data_path<<"mask/mask.png";
    string filename_mask = sstr_mask.str();

    vpImage<unsigned char> tmp_mask(m_width, m_height);
    m_vs_forward.read_mask(filename_mask, tmp_mask);
    m_vs_backward.read_mask(filename_mask, tmp_mask);

    tmp_mask.halfSizeImage(m_mask_image);

    m_vs_forward.init_visual_servo(m_sI, true, false, false, false, false, true);
    m_vs_backward.init_visual_servo(m_sI, true, false, false, false, false, true);
}


void PathManager::initValues()
{
    m_curr_vel_x = 9999.9;
}



void PathManager::robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
{

    m_logfile<<"################################ call to robot odometry callback"<<endl;
//    m_logfile<<odomsg->pose.pose.position.x<<"\t"<<odomsg->pose.pose.position.y<<"\t"<<odomsg->pose.pose.position.z<<endl;

    this->m_current_pose = *odomsg;
    m_logfile<<m_current_pose.pose.pose.position.x<<"\t"<<m_current_pose.pose.pose.position.y
            <<"\t"<<m_current_pose.pose.pose.position.z<<endl;

}


double PathManager::get_position_in_x(const nav_msgs::Odometry& odomsg)
{
    return odomsg.pose.pose.position.x;
}

void PathManager::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    m_logfile<<"################################ call to imageCallback"<<endl;

#if TAKE_REGLAR_FRAMES
    stringstream sstream;
    if (((m_iter % 50) == 0 ) || (m_iter == 0))
    {
        this->m_desired_image = visp_bridge::toVispImage(*image);
        // perform visual servoing => get the velocity
        build_desired_feature(m_desired_image);
        sstream<<m_logs_path<<"desired_images/desired_image_"<<std::setw(4) << std::setfill('0')<<m_iter<<".png";
        this->multiply_image_by_mask(m_desired_image, m_mask_image);
        vpImageIo::write(this->m_desired_image, sstream.str().c_str());
        vpDisplay::display(m_desired_image);

        vpDisplay::flush(m_desired_image);
        m_logfile<<"test_logfile in imageCallback"<<endl;
        m_index_desired_image = m_iter;
    }
    m_iter++;
#endif

#if TAKE_FRAMES_USING_VISUAL_SERVO
    stringstream sstream;
    m_iter++;
    vpImage<unsigned char> curr_im_temp(m_height, m_width);
    curr_im_temp = visp_bridge::toVispImage(*image);
    curr_im_temp.halfSizeImage(m_current_image);

    //    m_logfile<<"#### iteration"<<m_iter<<endl;
//    stringstream sstrm;
//    sstrm<<"/home/yalj/catkin_ws/src/my_package/logs/current_images_1/image_"<<std::setw(4)<<std::setfill('0')<<m_iter<<".png";
//    vpImageIo::write(curr_im_temp, sstrm.str().c_str());
//    m_velocity.linear.x = 0.1;
//    m_ros_pub.publish(m_velocity);
//    ros::spinOnce();

    //
    // convert
    //

    if (m_move_forward)
    {
        m_vs_forward.increment_iteration();
        m_vs_forward.multiply_image_by_mask(m_current_image, m_mask_image);

        m_logfile<<"#### iteration vs_forward"<<m_vs_forward.get_iteration_number()<<endl;

        m_logfile<<"################# MOVING FORWARD ############################"<<endl;
        if (m_save_desired_features)
        {

            m_prev_position    = m_current_position;
            m_current_position = get_position_in_x(m_current_pose);

            m_logfile<<"#### prev_position\t"<<m_prev_position<<endl;
            m_logfile<<"#### current_position\t"<<m_current_position<<endl;


            double diff_position = fabs(m_prev_position - m_current_position);
            m_log_odom<<m_index_desired_image_fwd<<"\t"<<diff_position<<endl;

            m_logfile<<"###################################  capturing desired image"<<m_index_desired_image_fwd<<endl;
            this->m_desired_image = this->m_current_image;
            sstream<<m_logs_path<<"desired_images/desired_image_"<<std::setw(4) << std::setfill('0')<<m_index_desired_image_fwd<<".png";

            std_msgs::String str;
            str.data  = sstream.str();
            this->m_ros_pub_im_filename.publish(str);

            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);

            m_logfile<<"display"<<endl;
            m_vec_desired_images.push_back(m_desired_image);

            this->m_prev_desired_image = m_vec_desired_images[0];

            m_sId = new CFeatureLuminanceOmni();
            m_logfile<<"before build desird feature"<<endl;

            m_vs_forward.build_desired_feature(m_desired_image,
                                               true, false, false,
                                               false, false, true, m_sId, m_Lsd, m_Hsd, m_diagHsd);

            m_logfile<<"m_index_desired_image_fwd\t"<<m_index_desired_image_fwd<<"\tm_diaghsd\t"<<m_diagHsd<<endl;
            m_logfile<<"computer desired features from desired image"<<endl;
            this->m_vec_Lsd.push_back(m_Lsd);
            this->m_vec_Hsd.push_back(m_Hsd);
            this->m_vec_diagHsd.push_back(m_diagHsd);
            this->m_vec_sId.push_back(m_sId);
            m_save_desired_features = false;


        }

        else
        {
            vpDisplay::display(m_current_image);
            vpDisplay::flush(m_current_image);

            vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
            vpDisplay::display(m_difference_image);
            vpDisplay::flush(m_difference_image);

            vpColVector v;

            this->m_prev_vel_x = m_curr_vel_x;
            this->m_prev_vel_y = m_curr_vel_y;

            m_vs_forward.perform_a_VS_iteration_without_MEstimator(m_sI, m_sId, m_current_image,
                                                                   v, m_Lsd, m_Hsd, m_diagHsd,
                                                                   m_logfile);
            this->m_curr_vel_x  = v[0];
            this->m_curr_vel_y  = v[1];


            m_logfile<<"curr_vel"<<m_curr_vel_x<<"\tm_prev_vel"<<m_prev_vel_x<<endl;
            m_logfile<<"velocity "<<v[0]<<"\t"<<v[1]<<endl;


            // iteration corresponding to 0 was consumed in "if (m_save_desired_features)"
            if (m_vs_forward.get_iteration_number() == 1)
            {
                this->m_first_it_vel_x = v[0];
                this->m_first_it_vel_y = v[1];
            }

            m_logerror<<m_vs_forward.get_iteration_number()<<"\t"<<m_vs_forward.getCurrentError()<<endl;
            m_log_vel<<m_iter<<"\t"<<v[0]<<endl;

            m_logfile<<"##### m_iter"<<m_iter<<"\tRatio\t"<<fabs(this->m_curr_vel_x/this->m_first_it_vel_x)<<"\t Diff\t"<<fabs(m_prev_vel_x -  m_curr_vel_x)<<endl;

            m_logfile<<"m_ratio_threshold\t"<<m_ratio_threshold<<"\tm_diff_threshold"<<m_diff_threshold<<"\tm_max_iters"<<m_max_iters<<endl;

            if ((fabs(this->m_curr_vel_x/this->m_first_it_vel_x> m_ratio_threshold) && (fabs(m_prev_vel_x -  m_curr_vel_x) <m_diff_threshold))
                    || (m_vs_forward.get_iteration_number() == m_max_iters)
                    //                        || ((fabs(this->m_curr_vel_y/this->m_first_it_vel_y)> m_ratio_threshold) && (fabs(m_prev_vel_y -  m_curr_vel_y) <2*m_diff_threshold))
                    )
            {
                m_save_desired_features = true;
                m_vs_forward.initialize();
                this->initValues();
                m_index_desired_image_fwd++;
            }

        }
    }

    else if (m_move_backward)
    {
        m_logfile<<"################# MOVING BACKWARD ############################"<<endl;
        this->m_vs_backward.increment_iteration();
        m_logfile<<"########################### iteration"<<m_vs_backward.get_iteration_number()<<endl;

        m_vs_backward.multiply_image_by_mask(m_current_image, m_mask_image);


        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

        vpColVector e ;
        vpColVector v ; // camera velocity send to the robot

        stringstream str_curr, str_diff;
        m_logfile<<"size desired images vector"<<m_vec_desired_images.size() <<endl;

        if (m_index_desired_image_bwd < this->m_vec_desired_images.size() )
        {
            this->m_desired_image = this->m_vec_desired_images[this->m_index_desired_image_bwd];
            vpDisplay::display(m_desired_image);
            vpDisplay::flush(m_desired_image);

            m_logfile<<"################################ Moving backward loading desired image n "<<m_index_desired_image_bwd<<endl;

            m_logfile<<"m_current_image"<<m_current_image.getCols()<<endl;
            m_logfile<<"m_desired_image"<<m_desired_image.getCols()<<endl;
            m_logfile<<"m_difference_image"<<m_difference_image.getCols()<<endl;

            vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
            vpDisplay::display(m_difference_image);
            vpDisplay::flush(m_difference_image);
            str_curr<<m_logs_path<<"current_images/current_image_"<<this->m_index_desired_image_bwd<<"_"<<m_vs_backward.get_iteration_number()<<".png";
            vpImageIo::write(m_current_image,str_curr.str());

            str_diff<<m_logs_path<<"diff_images/diff_image_"<<this->m_index_desired_image_bwd<<"_"<<m_vs_backward.get_iteration_number()<<".png";
            vpImageIo::write(m_difference_image,str_diff.str());

            double t = vpTime::measureTimeMs();
            m_logfile<<"m_index_desired_image_bwd\t"<<m_index_desired_image_fwd<<"\tm_diaghsd\t"<<m_vec_diagHsd[m_index_desired_image_bwd]<<endl;



            m_logfile<<"### Current desired image\t"<<m_index_desired_image_bwd<<endl;
            double angle_degrees;
            int step = 2;
            double angle_radians;
            double lambda = 0.25;
            double omega;

            // get the previous desired image when it is available
            if ((m_index_desired_image_bwd >= 1)  && (m_index_desired_image_bwd < this->m_vec_desired_images.size()- 1))
            {

                this->m_prev_desired_image = this->m_vec_desired_images[this->m_index_desired_image_bwd-1];
                vpDisplay::display(m_prev_desired_image);
                vpDisplay::flush(m_prev_desired_image);

                //
                // compute angle between the desired and previous image
                //
                angle_degrees = m_vs_backward.find_best_angle(m_prev_desired_image, m_desired_image, m_center_rotation, step);
                angle_radians = angle_degrees*M_PI/180.0;

                m_logfile<<"angle degrees betwwen desired and previous"<<angle_degrees<<endl;

                if ((fabs(angle_degrees) > 5) && m_flag)
                {
                    m_flag = false;
                    m_rotation_mode = true;
                    m_translation_mode = false;
                }
            }

            if (m_rotation_mode)
            {

                m_logfile<<"rotation mode"<<endl;
                // rotation mode
                double theta_degrees = m_vs_backward.find_best_angle(m_desired_image, m_current_image , m_center_rotation, step);

                m_logfile<<"theta degrees\t"<<theta_degrees<<endl;
                //                    double theta_radians = theta_degrees*M_PI/180.0;
                double theta_radians = vpMath::rad(theta_degrees);

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
                    m_velocity.angular.z = omega+ vpMath::sign(omega)*0.05;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }

            }

            if (m_translation_mode && (m_index_desired_image_bwd < this->m_vec_desired_images.size()))
            {
                m_logfile<<"m_index_desired_image_bwd\t"<<m_index_desired_image_bwd<<endl;
                m_vs_backward.perform_a_VS_iteration_without_MEstimator(m_sI,this->m_vec_sId[m_index_desired_image_bwd],
                                                                        m_current_image,
                                                                        v, this->m_vec_Lsd[m_index_desired_image_bwd],
                                                                        m_vec_Hsd[m_index_desired_image_bwd],
                                                                        m_vec_diagHsd[m_index_desired_image_bwd], m_logfile);



                t = vpTime::measureTimeMs() - t;
                m_logfile << "Time per frame "<<t<< std::endl;
                m_logfile << "### Velocity "<<v.t()<< std::endl;
                m_logfile << "### Iteration "<<m_vs_backward.get_iteration_number()<< std::endl;
                m_logfile << "### m_threshold_error "<<m_threshold_error<< std::endl;
                m_logerror<<m_iter<<"\t"<<m_vs_backward.getCurrentError()<<endl;

                if ( (this->m_vs_backward.getCurrentError() < m_threshold_error)  || (m_vs_backward.get_iteration_number() > 40) )
                {
                    if ((v[0]>0.0)   )
                    {
                        m_index_desired_image_bwd++;
                        m_flag = true;
                        m_vs_backward.initialize();
                        m_vs_backward.init_params_without_MEstimator(m_logfile);
                    }

                    else
                    {
                        if (v[0] > -0.1 && v[0]<0.0)
                        {
                            v[0] = -0.1;
                        }
                        m_velocity.linear.x = v[0];
                        m_velocity.angular.z = v[1];
                        m_ros_pub.publish(m_velocity);
                        ros::spinOnce();
                    }
                }

                else if (v[0] > 0.0)
                {
                    m_velocity.linear.x = 0.0;
                    m_velocity.angular.z = 0.0;
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }
                else
                {
                    if (v[0]> -0.1 && v[0]<0.0 )
                    {
                        v[0] = -0.1;
                    }
                    m_logfile<<" in if else"<<endl;
                    m_velocity.linear.x = v[0];
                    m_velocity.angular.z = v[1];
                    m_ros_pub.publish(m_velocity);
                    ros::spinOnce();
                }
            }


            //                else
            //                {
            //                }
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


#endif
}


void PathManager::reverse_vectors()
{
    m_logfile<<"##### reverting vectors"<<endl;
    std::reverse(m_vec_desired_images.begin(), m_vec_desired_images.end());
    std::reverse(m_vec_sId.begin(), m_vec_sId.end());
    std::reverse(m_vec_Lsd.begin(), m_vec_Lsd.end());
    std::reverse(m_vec_Hsd.begin(), m_vec_Hsd.end());
    std::reverse(m_vec_diagHsd.begin(), m_vec_diagHsd.end());
}



void PathManager::write_data()
{
    //write desired image
    for(int i= 0; i<this->m_vec_desired_images.size();i++)
    {
        stringstream sstream;
        sstream<<m_logs_path<<"desired_images/desired_image_"<<std::setw(4) << std::setfill('0')<<i<<".png";
        vpImageIo::write(this->m_vec_desired_images[i], sstream.str().c_str());
    }

    m_logfile<<"size desired images"<<m_vec_desired_images.size()<<endl;

    //write matrices

    for(int i= 0; i<this->m_vec_desired_images.size();i++)
    {
        m_vs_forward.write_matrices(m_vec_Lsd[i], m_vec_Hsd[i], m_vec_diagHsd[i], i, m_data_path);
    }
}


void PathManager::robotVelocityCallback(const geometry_msgs::TwistConstPtr& twist_vs)
{
    m_listened_velocity.linear.x = twist_vs->linear.x;
    m_listened_velocity.angular.z = twist_vs->angular.z;
}


void PathManager::callbackTimer(const ros::TimerEvent&)
{
    m_logfile<<"we havent received any velocity "<<endl;
    if ((m_listened_velocity.linear.x == 0.0) && (m_listened_velocity.angular.z == 0.0) && (m_vs_forward.get_iteration_number() !=0))
    {

        m_logfile<<"we havent received any velocity for 10 seconds"<<endl;
        if (m_nb_backward == 0)
        {
            m_logfile<<"speed is null"<<endl;

            //
            // reverse order of vectors
            //
            write_data();

            m_logfile<<"#### reverting vectors"<<endl;
            reverse(m_vec_desired_images.begin(), m_vec_desired_images.end());
            reverse(m_vec_Lsd.begin(), m_vec_Lsd.end());
            reverse(m_vec_Hsd.begin(), m_vec_Hsd.end());
            reverse(m_vec_diagHsd.begin(), m_vec_diagHsd.end());
            reverse(m_vec_sId.begin(), m_vec_sId.end());

            m_vs_backward.init_params_without_MEstimator(m_logfile);
            m_move_forward  = false;
            m_move_backward = true;
        }
        m_nb_backward++;
    }
    else
    {
        m_logfile<<"m_listened_velocity\t"<<m_listened_velocity<<endl;
    }
}
