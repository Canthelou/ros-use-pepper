#include "VisualServoTools.h"


#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include "visp_bridge/image.h"


using namespace std;

class VisualPathFollowing
{

public:

    VisualPathFollowing()
        : m_it(m_nh)
    {

        m_move_forward = true;
        m_move_backward = false;
        m_vs_translation = false;

        m_nb_tours = 0;
        m_mean_diff_error = 0.0;
        m_diff_error = 0.0;
        m_first_forward  = 0;
        m_first_backward = 0;

        m_width     = m_visual_servo_tools.getWidth();
        m_height    = m_visual_servo_tools.getHeight();

        m_center_image[0] = m_height/2;
        m_center_image[1] = m_width/2;

        m_center_rotation[0] = m_u0;
        m_center_rotation[1] = m_v0;


        m_nb_croissance = 0;
        m_index_desired_image_fwd  = 0;
        m_index_desired_image_bwd  = 0;
        string cameraTopic, robotTopic, visualServoMovements;

        m_nh.param("cameraTopic", cameraTopic, string(""));
        m_nh.param("robotTopic", robotTopic, string(""));

        m_nh.param("visualServoMovements", visualServoMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path,string(""));
        m_nh.param("error_threshold",m_threshold_error, double(0.0));

        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("visualServoMovements", visualServoMovements);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        m_nh.getParam("error_threshold",m_threshold_error);
        m_nh.getParam("robotTopic", robotTopic);

        m_px  = m_visual_servo_tools.getPx();
        m_py  = m_visual_servo_tools.getPy();
        m_u0  = m_visual_servo_tools.getU0();
        m_v0  = m_visual_servo_tools.getV0();
        m_xi  = m_visual_servo_tools.getXi();
        m_rho = m_visual_servo_tools.getRHO();



        this->initValues();

        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        stringstream str_error;
        str_error<<m_logs_path<<"error.mat";
        m_logerror.open(str_error.str().c_str());

        m_current_image.init(m_height, m_width);
        m_desired_image.init(m_height, m_width);
        m_difference_image.init(m_height, m_width);
        m_prev_desired_image.init(m_height, m_width);

        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
        m_display_prev_desired_image.init(this->m_prev_desired_image, m_width+10, m_height+30, "Prev Desired image");
        m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");

        initVisualPathFollowing();

        m_image_sub = m_it.subscribe(cameraTopic, 1, &VisualPathFollowing::imageCallback, this);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1000);

        m_flag  = true;
        m_translation_mode = false;
        m_rotation_mode = true;
    }

    ~VisualPathFollowing()
    {
        m_logfile.close();

        for (int i=0; i<m_vec_sId.size();i++)
        {
            delete m_vec_sId[i];
        }
    }

    void initValues()
    {
        //        m_iter      = 0;
        //        m_mu        = 0.001;
        //        m_lambda  = 20;

        m_iter                = 0;
        //        m_mu                  = 0.00001;
        //        m_lambda              = 105;
        //        m_nb_croissance       = 0;
        //        m_mean_diff_error     = 0.0;
        //        m_diff_error          = 0.0;
    }

    void initVisualPathFollowing()
    {
        m_logfile<<"#### Threshold error"<<m_threshold_error<<endl;

        //
        // Load image mask
        //
        stringstream  sstr_mask;
        sstr_mask<<m_data_path<<"mask/mask.png";
        string filename_mask = sstr_mask.str();

        vpImage<unsigned char> tmp_mask(m_width, m_height);
        m_visual_servo_tools.initialize();
        m_visual_servo_tools.init_params_with_MEstimator(m_logfile);

        m_visual_servo_tools.read_mask(filename_mask, tmp_mask);
        tmp_mask.halfSizeImage(m_mask_image);

        //        m_visual_servo_tools.read_mask(filename_mask, m_mask_image);
        //        m_logfile<<"filename_mask"<<filename_mask<<endl;
        //        vpImageIo::read(this->m_mask_image, filename_mask);


        // Load desired images
        stringstream str;
        str<<m_logs_path<<"desired_images/";
        string filename_prefix = "desired_image";
        std::vector<string> filenames_desired_images, filenames_interaction, filenames_Hsd, filenames_diagHsd;
        m_visual_servo_tools.load_directory_and_sort_files(str.str().c_str(), filename_prefix.c_str(), filenames_desired_images);

        // Load interaction matrice files
        stringstream str_inter_mat_file;
        str_inter_mat_file<<m_data_path<<"interaction/";
        string filename_prefix_inter = "interaction";
        m_visual_servo_tools.load_directory_and_sort_files(str_inter_mat_file.str().c_str(), filename_prefix_inter.c_str(), filenames_interaction);

        // Load Hsd matrice files
        stringstream str_Hsd_file;
        str_Hsd_file<<m_data_path<<"Hsd/";
        string filename_prefix_Hsd = "Hsd";
        m_visual_servo_tools.load_directory_and_sort_files(str_Hsd_file.str().c_str(), filename_prefix_Hsd.c_str(), filenames_Hsd);

        // Load Hsd matrice files
        stringstream str_diagHsd_file;
        str_diagHsd_file<<m_data_path<<"diagHsd/";
        string filename_prefix_diagHsd = "diagHsd";
        m_visual_servo_tools.load_directory_and_sort_files(str_diagHsd_file.str().c_str(), filename_prefix_diagHsd.c_str(), filenames_diagHsd);

        this->m_desired_images = std::vector<vpImage<unsigned char> > (filenames_desired_images.size());

        //
        // read desired images
        //
        for (unsigned int i = 0;  i< filenames_desired_images.size(); i++)
        {

            stringstream  sstr_des;
            string filename;

            m_desired_images[i].init(this->m_height, m_width);
            sstr_des <<m_logs_path<<"desired_images/"<<filenames_desired_images[i];
            filename =sstr_des.str();
            vpImageIo::read(m_desired_images[i],filename);

            vpDisplay::display(m_desired_images[i]);
            vpDisplay::flush(m_desired_images[i]);

            CFeatureLuminanceOmni *sId = new CFeatureLuminanceOmni();

            vpMatrix Lsd, Hsd, diagHsd;

            m_visual_servo_tools.build_desired_feature(this->m_desired_images[i],
                                                       true, false, false,
                                                       false, false, true,
                                                       sId,Lsd,Hsd,diagHsd);
            m_logfile<<"building desired feature "<<i<<endl;

            m_vec_sId.push_back(sId);
            m_vec_Lsd.push_back(Lsd);
            m_vec_Hsd.push_back(Hsd);
            m_vec_diagHsd.push_back(diagHsd);
        }

        m_prev_desired_image = m_desired_images[0];


        m_visual_servo_tools.init_visual_servo(m_sI, true, false, false,
                                               false, false, true);

    }


    void reverse_vectors()
    {
        m_logfile<<"##### reverting vectors"<<endl;
        std::reverse(m_desired_images.begin(), m_desired_images.end());
        std::reverse(m_vec_sId.begin(), m_vec_sId.end());
        std::reverse(m_vec_Lsd.begin(), m_vec_Lsd.end());
        std::reverse(m_vec_Hsd.begin(), m_vec_Hsd.end());
        std::reverse(m_vec_diagHsd.begin(), m_vec_diagHsd.end());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {

        //
        // acquire image
        //

        //        this->m_current_image = visp_bridge::toVispImage(*image);

        vpImage<unsigned char> curr_im_temp(m_height, m_width);
        curr_im_temp = visp_bridge::toVispImage(*image);
        curr_im_temp.halfSizeImage(m_current_image);


        //
        // rotate image
        //
//        m_visual_servo_tools.rotate_my_image(m_current_image, this->m_center_image, -90.0, m_current_image);

        m_visual_servo_tools.multiply_image_by_mask(m_current_image, m_mask_image);
        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

        vpColVector e ;
        vpColVector v ; // camera velocity send to the robot
        stringstream str_curr,str_diff;

        if (m_move_forward)
        {
            m_logfile<<"### moving forward"<<endl;
            if (m_index_desired_image_fwd < this->m_desired_images.size() )
            {

                double angle_degrees;
                int step = 2;
                double angle_radians;
                double lambda = 0.25;
                double omega;

                this->m_desired_image = this->m_desired_images[this->m_index_desired_image_fwd];

                vpDisplay::display(m_desired_image);
                vpDisplay::flush(m_desired_image);


                m_logfile<<"################################ Moving backward loading desired image n "<<m_index_desired_image_bwd<<endl;

                vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
                vpDisplay::display(m_difference_image);
                vpDisplay::flush(m_difference_image);
                str_curr<<m_logs_path<<"current_images/current_image_"<<this->m_index_desired_image_bwd<<"_"<<m_visual_servo_tools.get_iteration_number()<<".png";
                vpImageIo::write(m_current_image,str_curr.str());

                str_diff<<m_logs_path<<"diff_images/diff_image_"<<this->m_index_desired_image_bwd<<"_"<<m_visual_servo_tools.get_iteration_number()<<".png";
                vpImageIo::write(m_difference_image,str_diff.str());

                double t = vpTime::measureTimeMs();
                m_logfile<<"m_index_desired_image_bwd\t"<<m_index_desired_image_fwd<<"\tm_diaghsd\t"<<m_vec_diagHsd[m_index_desired_image_bwd]<<endl;

//                if (m_translation_mode)
//                {
                    m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI, m_vec_sId[m_index_desired_image_fwd],
                                                                                m_current_image, v,
                                                                                m_vec_Lsd[m_index_desired_image_fwd],
                                                                                m_vec_Hsd[m_index_desired_image_fwd],
                                                                                m_vec_diagHsd[m_index_desired_image_fwd],
                                                                                m_logfile);

                    t = vpTime::measureTimeMs() - t;

                    m_logfile << "Time per frame "<<t<< std::endl;
                    m_logfile << "### Velocity "<<v.t()<< std::endl;
                    m_logfile << "### Iteration "<<m_iter<< std::endl;
                    m_iter++;

                    if ( this->m_visual_servo_tools.getCurrentError() < 0.016)
                    {
                        if ((v[0]<0.0) )
                        {
                            m_index_desired_image_fwd++;
                            m_flag = true;
                            m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                            m_visual_servo_tools.initialize();
                        }

                        else
                        {
                            if (v[0] < 0.1 )
                            {
                                v[0] = 0.1;
                            }
                            m_velocity.linear.x  = v[0];
                            m_velocity.angular.z = v[1];
                            m_ros_pub.publish(m_velocity);
                            ros::spinOnce();
                        }
                    }

                    else
                    {
                        if (v[0]< 0.1 )
                        {
                            v[0] = 0.1;
                        }
                        m_velocity.linear.x  = v[0];
                        m_velocity.angular.z = v[1];
                        m_ros_pub.publish(m_velocity);
                        ros::spinOnce();
                    }
//                }

            }

            else
            {

                m_velocity.linear.x  = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();

            }
        }

        /*
        if (m_move_forward)
        {
            m_logfile<<"### moving forward"<<endl;
            if (m_index_desired_image_fwd < this->m_desired_images.size() )
            {
                this->m_desired_image = this->m_desired_images[this->m_index_desired_image_fwd];

                vpDisplay::display(m_desired_image);
                vpDisplay::flush(m_desired_image);

                vpDisplay::display(m_prev_desired_image);
                vpDisplay::flush(m_prev_desired_image);
                // if the angle between desired and previous desired image is above a certain thershold turn the robot

                int step  = 2;
                double theta_degrees, theta_radians, omega, theta_temp_degrees, theta_temp_radians;
                double lambda = 0.25;

                theta_degrees = m_visual_servo_tools.find_best_angle(m_prev_desired_image, m_desired_image, m_center_rotation, step, m_logfile);

                theta_radians = theta_degrees*M_PI/180.0;

                m_logfile<<"theta_degrees"<<theta_degrees<<endl;

                if ((theta_degrees > 10) )
                {
                    m_logfile<<"theta above 20"<<endl;
                    m_pure_rotation = true;
                    m_vs_translation = false;
                }
                else if (theta_degrees < 10)
                {
                    m_logfile<<"theta below 20"<<endl;
                    m_pure_rotation = false;
                    m_vs_translation = true;
                }



                if (m_pure_rotation)
                {

                    //
                    // compute angle between current and desired image until differenc  e is almost zero
                    //

                    m_logfile<<"###### PURE ROTATION"<<endl;
                    theta_temp_degrees  = this->m_visual_servo_tools.find_best_angle(m_desired_image, m_current_image, m_center_rotation, step, m_logfile);

                    theta_temp_radians = theta_temp_degrees*M_PI/180.0;


                    m_logfile<<"Degrees : required\t"<<theta_degrees<<"\tvisual_compass:\t"<<theta_temp_degrees<<endl;

                    omega = -lambda*(theta_temp_radians - theta_radians);

                    m_velocity.linear.x = 0.0;
                    m_logfile<<"velocity\t"<<v<<endl;
                    m_logfile<<"angular velocity"<<omega<<endl;

                    //            m_vs_first_rot.clean();

                    //            if (fabs(omega) < 2e-2)
                    if (fabs(theta_temp_degrees - theta_degrees) < 3.)
                    {
                        m_logfile<<"omega is almost zero"<<endl;
                        m_vs_translation = true;
                        m_pure_rotation = false;
                        m_velocity.angular.z = 0.0;
                        m_ros_pub.publish(m_velocity);

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

                if (m_vs_translation)
                {
                    m_logfile<<"translation pure"<<endl;
                    double t = vpTime::measureTimeMs();

                    m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI, m_vec_sId[m_index_desired_image_fwd],
                                                                                m_current_image, v,
                                                                                m_vec_Lsd[m_index_desired_image_fwd],
                                                                                m_vec_Hsd[m_index_desired_image_fwd],
                                                                                m_vec_diagHsd[m_index_desired_image_fwd],
                                                                                m_logfile);

                    t = vpTime::measureTimeMs() - t;

                    m_logfile << "Time per frame "<<t<< std::endl;
                    m_logfile << "### Velocity "<<v.t()<< std::endl;
                    m_logfile << "### Iteration "<<m_iter<< std::endl;
                    m_iter++;

                    if ( this->m_visual_servo_tools.getCurrentError() < 0.016 )
                    {
                        if ((v[0]<0.0) )
                        {
                            this->m_prev_desired_image = this->m_desired_images[this->m_index_desired_image_fwd];

                            m_index_desired_image_fwd++;
                            m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                            m_visual_servo_tools.initialize();
                        }

                        else
                        {
                            if (v[0] < 0.1 )
                            {
                                v[0] = 0.1;
                            }
                            m_velocity.linear.x = v[0];
                            m_velocity.angular.z = v[1];
                            m_ros_pub.publish(m_velocity);
                            ros::spinOnce();
                        }
                    }


                    else
                    {
                        if (v[0]< 0.1 )
                        {
                            v[0] = 0.1;
                        }
                        m_velocity.linear.x = v[0];
                        m_velocity.angular.z = v[1];
                        m_ros_pub.publish(m_velocity);
                        ros::spinOnce();
                    }
                }
                //            m_visual_servo_tools.multiply_image_by_mask(m_desired_image, m_mask_image);
                m_logfile<<"################################ loading desired image n "<<m_index_desired_image_fwd <<endl;
                vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
                vpDisplay::display(m_difference_image);
                vpDisplay::flush(m_difference_image);

                this->m_current_images.push_back(m_current_image);
                this->m_diff_images.push_back(m_difference_image);
            }

            else
            {

                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
//                m_move_forward = false;
//                m_move_backward = true;
//                m_index_desired_image_bwd = 0;
//                reverse_vectors();
//                m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
//                m_visual_servo_tools.initialize();
            }
        }

*/


        else if (m_move_backward)
        {
            m_logfile<<"### Moving backward"<<endl;
            if (m_index_desired_image_bwd < this->m_desired_images.size() )
            {
                this->m_desired_image = this->m_desired_images[this->m_index_desired_image_bwd];
                //            m_visual_servo_tools.multiply_image_by_mask(m_desired_image, m_mask_image);
                vpDisplay::display(m_desired_image);
                vpDisplay::flush(m_desired_image);

                m_logfile<<"################################ loading desired image n "<<m_index_desired_image_bwd <<endl;
                vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image);
                vpDisplay::display(m_difference_image);
                vpDisplay::flush(m_difference_image);

                this->m_current_images.push_back(m_current_image);
                this->m_diff_images.push_back(m_difference_image);

                double t = vpTime::measureTimeMs();

                m_visual_servo_tools.perform_a_VS_iteration_with_MEstimator(m_sI, m_vec_sId[m_index_desired_image_bwd],
                                                                            m_current_image,v,
                                                                            m_vec_Lsd[m_index_desired_image_bwd],
                                                                            m_vec_Hsd[m_index_desired_image_bwd],
                                                                            m_vec_diagHsd[m_index_desired_image_bwd],
                                                                            m_logfile);

                t = vpTime::measureTimeMs() - t;

                m_logfile << "Time per frame "<<t<< std::endl;
                m_logfile << "### Velocity "<<v.t()<< std::endl;
                m_logfile << "### Iteration "<<m_iter<< std::endl;
                m_iter++;

                if ( (m_visual_servo_tools.getCurrentError() < m_threshold_error) )
                {
                    if ((v[0]>0.0) /*|| m_iter > 40*/)
                    {
                        m_index_desired_image_bwd++;
                        m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
                        m_visual_servo_tools.initialize();

                    }

                    else
                    {
                        if (v[0] > -0.1 && v[0]<0.0)
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

            else
            {
                m_move_forward = true;
                m_move_backward = false;
                m_index_desired_image_fwd = 0;
                //                m_index_desired_image_fwd = 0;

                reverse_vectors();
                initValues();
            }

        }





    }



    /*
        if ( (this->m_current_error < m_threshold_error) && (m_index_desired_image_fwd <= this->m_desired_images.size()) )
        {
            m_logfile<<"error is smaller"<<endl;
            if ((v[0]<0.0) )
            {

                m_index_desired_image_fwd++;
//                increment();
                initValues();
            }

            else
            {
                if (v[0] < 0.1 )
                {
                    v[0] = 0.1;
                }
                m_velocity.linear.x = v[0];
                m_velocity.angular.z = v[1];
                m_ros_pub.publish(m_velocity);
                ros::spinOnce();
            }
        }

        else if (m_index_desired_image_fwd > this->m_desired_images.size())
        {
            m_velocity.linear.x = 0.0;
            m_velocity.angular.z = 0.0;
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
        }
        else
        {
            if (v[0]< 0.1 )
            {
                v[0] = 0.1;
            }
            m_velocity.linear.x = v[0];
            m_velocity.angular.z = v[1];
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
        }

        */
    //    }






















    //        m_index_desired_image_bwd = this->m_desired_images.size() -1;

    //        for (unsigned int i = 0; i <filenames_desired_images.size();i++ )
    //        {

    //vpMatrix Lsd, Hsd, diagHsd;

    //            sId->setCameraParameters(m_visual_servo_tools.getCamParams()) ;
    //            sId->setInterpType(INTERPTYPE);
    //            sId->init(this->m_height, this->m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_visual_servo_tools.getRHO(), REPTYPE, GRAPCALCTYPE) ;
    //            sId->set_DOF(true, false, false, //Vx Vy Vz
    //                         false, false, true); //Wx Wy Wz
    //            //
    //            // build visual feature from desired image
    //            //
    //            sId->buildFrom(this->m_desired_images[i]);
    //            //
    //            // Compute interaction matrix at desired position
    //            //
    //            sId->interaction(Lsd) ;
    //            // Compute the Hessian H = L^TL
    //            Hsd = Lsd.AtA() ;
    //            // Compute the Hessian diagonal for the Levenberg-Marquardt
    //            // optimization process
    //            unsigned int n = Lsd.getCols() ;
    //            diagHsd = vpMatrix(n,n) ;
    //            diagHsd.eye(n);
    //            for(unsigned int ii = 0 ; ii < n ; ii++)
    //            {
    //                diagHsd[ii][ii] = Hsd[ii][ii];
    //            }

    //            m_visual_servo_tools.init_visual_servo();

    //            m_vec_sId.push_back(sId);
    //            m_vec_Lsd.push_back(Lsd);
    //            m_vec_Hsd.push_back(Hsd);
    //            m_vec_diagHsd.push_back(diagHsd);
    //        }

    //        point_to_begin();
    //        point_to_end();
    //        decrement();

    //        m_visual_servo_tools.init_visual_servo(m_sI, true, false, false,
    //                                               false, false, true);

    //        m_index_desired_image_bwd = this->m_desired_images.size() -1;


    //        m_sI.init(this->m_height, this->m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_visual_servo_tools.getRHO(), REPTYPE, GRAPCALCTYPE);
    //        m_sI.setCameraParameters(m_visual_servo_tools.getCamParams()) ;
    //        m_sI.set_DOF(true, false, false, //Vx Vy Vz
    //                     false, false, true); //Wx Wy Wz

    //        m_sI.setInterpType(INTERPTYPE);

    //        m_index_desired_image_bwd = this->m_desired_images.size() -1;





    //        str_curr<<m_logs_path<<"current_images/current_image_"<<this->m_index_desired_image_fwd -1<<"_"<<m_iter<<".png";
    //        vpImageIo::write(m_current_image,str_curr.str());

    //        str_diff<<m_logs_path<<"diff_images/diff_image_"<<this->m_index_desired_image_fwd -1<<"_"<<m_iter<<".png";
    //        vpImageIo::write(m_difference_image,str_diff.str());

    //        vpRobust robust(0);
    //        robust.setThreshold(0.0);

    //        vpColVector w;
    //        vpColVector weighted_error;


    //        //            vpColVector v;

    //        //            vpColVector v;
    //        //            m_visual_servo_tools.perform_a_VS_iteration_w(m_sI,m_sId,m_current_image,m_iter,m_lambda, m_mu,
    //        //                                                          v,m_Lsd, m_Hsd, m_diagHsd, m_current_error, m_pred_error, m_logfile);

    //        this->m_sI.buildFrom(m_current_image) ;

    //        // ------------------------------------------------------
    //        // Control law
    //        //

    //        // ----------------------------------------------------------
    //        // Minimisation
    //        double lambdaGN = 55.0;
    //        //            double lambdaGN = 25.0;
    //        // ----------------------------------------------------------
    //        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations


    //        double normeError = 0;

    //        m_logfile<<"avant calcul error"<<endl;
    //        // compute current error
    //        this->m_sI.error(*(*m_it_vec_sId),this->m_error);
    //        w.resize(this->m_error.getRows());
    //        weighted_error.resize(m_error.getRows());

    //        m_logfile<<"this->m_error.getCols()"<<this->m_error.getRows()<<endl;
    //        m_logfile<<"(*m_it_vec_Lsd).getRows()"<<(*m_it_vec_Lsd).getRows()<<endl;

    //        w =1;

    //        robust.MEstimator(vpRobust::TUKEY, m_error, w);
    //        //            m_logfile<<"weight "<<w<<endl;
    //        m_logfile<<"après calcul error"<<endl;

    //        // ---------- Levenberg Marquardt method --------------

    //        if (m_iter == iterGN)
    //        {
    //            m_mu =  0.000001 ;
    //            //                m_mu =  0.01 ;

    //            m_lambda = lambdaGN;
    //        }

    //        vpMatrix H;

    //        if (m_current_error < this->m_pred_error && m_iter > iterGN)
    //        {
    //            m_mu /=2;
    //        }

    //        //            if((m_current_error > this->m_pred_error) && (m_iter == (iter_croissance+1)))
    //        //            {
    //        //                iter_croissance = m_iter;
    //        //                m_logfile<<"increment m_nb_croissance"<<endl;
    //        //                m_nb_croissance++;
    //        //            }

    //        H = ((m_mu * (*m_it_diagHsd)) +(*m_it_vec_Hsd)).inverseByLU();
    //        //            m_logfile<<"après calcul de H"<<endl;

    //        vpMatrix interaction_matrix ((*m_it_vec_Lsd).getRows(),(*m_it_vec_Lsd).getCols());//= (*m_it_vec_Lsd);
    //        int nb_total_points = (*m_it_vec_Lsd).getRows();

    //        for (int i=0; i<nb_total_points; i++)
    //        {
    //            weighted_error[i] = w[i]*m_error[i];
    //            interaction_matrix[i][0] = (*m_it_vec_Lsd)[i][0]*w[i];
    //            interaction_matrix[i][1] = (*m_it_vec_Lsd)[i][1]*w[i];
    //        }

    //        //            m_logfile<<"new weighted error"<<endl;
    //        //            for(int j=0; j<nb_total_points;j++)
    //        //            {
    //        //            }
    //        //            m_logfile<<"new interaction matrix"<<endl;


    //        //	compute the control law

    //        //  m_logfile<<"(*m_it_vec_Lsd)"<<endl<<(*m_it_vec_Lsd)<<endl;

    //        //            e = H * (*m_it_vec_Lsd).t() *this->m_error;
    //        e = H * interaction_matrix.t() * weighted_error;

    //        //            m_logfile<<"weighter_error"<<weighted_error<<endl;
    //        //            m_logfile<<"interaction_matrix "<<interaction_matrix<<endl;

    //        this->m_pred_error = this->m_current_error;
    //        //            normeError = (this->m_error.sumSquare());
    //        normeError = weighted_error.sumSquare();
    //        this->m_current_error = sqrt(normeError)/(this->m_height*this->m_width);
    //        m_logfile << "### |e| "<< m_current_error<<std::endl ;
    //        m_logfile <<"error threshold"<<this->m_threshold_error<<endl;
    //        //            m_logfile <<"nb decroissance error"<<m_nb_croissance<<endl;
    //        m_logerror<<m_current_error<<endl;

    //        v = - m_lambda*e;

    //        t = vpTime::measureTimeMs() - t;
    //        m_logfile << "Time per frame "<<t<< std::endl;
    //        m_logfile << "### Velocity "<<v.t()<< std::endl;
    //        m_logfile << "### Iteration "<<m_iter<< std::endl;
    //        m_iter++;

    //        if (m_pred_error != 9999)
    //        {
    //            m_diff_error = fabs(m_current_error - m_pred_error);
    //            m_logfile<<"m_diff_error"<<m_diff_error<<endl;
    //            m_mean_diff_error += m_diff_error;
    //            m_logfile<<"cumulated diff error"<<m_mean_diff_error<<endl;
    //            m_mean_diff_error = m_mean_diff_error/(m_iter -1);
    //            m_logfile<<"m_mean_diff_error mean"<<m_mean_diff_error<<endl;
    //        }


    //    }

    //    if ( (this->m_current_error < m_threshold_error) && (m_index_desired_image_fwd <= this->m_desired_images.size()))
    //    {
    //        m_logfile<<"error is smaller"<<endl;
    //        if ((v[0]<0.0) || (m_iter > 40))
    //        {

    //            m_index_desired_image_fwd++;
    //            increment();
    //            initValues();
    //            m_logfile<<"# move forward  desired image in if if "<<m_index_desired_image_fwd<<endl;
    //        }

    //        else
    //        {
    //            if (v[0] < 0.1 )
    //            {
    //                v[0] = 0.1;
    //            }

    //            //                if( (v[1] < 0.1) && (v[1] > 0.005))
    //            //                {
    //            //                    v[1] = 0.1;
    //            //                }
    //            //                m_logfile << "lambda = " << m_lambda << "  mu = " << m_mu <<endl;
    //            //                m_logfile<<"v "<<v.t()<<endl;

    //            m_velocity.linear.x = v[0];
    //            m_velocity.angular.z = v[1];
    //            m_ros_pub.publish(m_velocity);
    //            ros::spinOnce();
    //        }
    //    }

    //    //        else if (m_index_desired_image_fwd > this->m_desired_images.size())
    //    //        {
    //    //            //            m_logfile<<"##### reaching desired position forward"<<m_index_desired_image_fwd<<endl;

    //    //            //            this->m_first_backward = 0;
    //    //            //            decrement();
    //    //            m_logfile<<"calling move_backward"<<endl;
    //    //            initValues();
    //    //            move_backward(image);
    //    //            m_first_backward++;

    //    //        }

    //    else if (m_index_desired_image_fwd > this->m_desired_images.size())
    //    {
    //        m_velocity.linear.x = 0.0;
    //        m_velocity.angular.z = 0.0;
    //        m_ros_pub.publish(m_velocity);
    //        ros::spinOnce();

    //    }


    //    else
    //{
    //    if (v[0]< 0.1 )
    //    {
    //        v[0] = 0.1;
    //    }

    //    //            if( v[1] < 0.1 && v[1] > 0.01)
    //    //            {
    //    //                v[1] = 0.1;
    //    //            }
    //    //            m_logfile << "lambda = " << m_lambda << "  mu = " << m_mu <<endl;
    //    //            m_logfile<<"v "<<v.t()<<endl;

    //    m_logfile<<"move forward in else"<<endl;
    //    m_velocity.linear.x = v[0];
    //    m_velocity.angular.z = v[1];
    //    m_ros_pub.publish(m_velocity);
    //    ros::spinOnce();
    //}
    //}


    //    void point_to_begin()
    //    {
    //        this->m_it_vec_sId = m_vec_sId.begin();
    //        this->m_it_vec_Lsd = m_vec_Lsd.begin();
    //        this->m_it_vec_Hsd = m_vec_Hsd.begin();
    //        this->m_it_diagHsd = m_vec_diagHsd.begin();
    //    }

    //    void point_to_end()
    //    {
    //        this->m_it_vec_sId_backward = m_vec_sId.end();
    //        this->m_it_vec_Lsd_backward = m_vec_Lsd.end();
    //        this->m_it_vec_Hsd_backward = m_vec_Hsd.end();
    //        this->m_it_diagHsd_backward = m_vec_diagHsd.end();
    //    }

    //    void increment()
    //    {
    //        this->m_it_vec_sId++;
    //        this->m_it_vec_Lsd++;
    //        this->m_it_vec_Hsd++;
    //        this->m_it_diagHsd++;
    //    }

    //    void decrement()
    //    {
    //        this->m_it_vec_sId_backward--;
    //        this->m_it_vec_Lsd_backward--;
    //        this->m_it_vec_Hsd_backward--;
    //        this->m_it_diagHsd_backward--;
    //    }





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
    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_mask_image;

    ofstream m_logfile;
    ofstream m_logerror;


    std::vector<string> m_vec_str_Ls;
    std::vector<string> m_vec_str_Hs;
    std::vector<string> m_vec_str_diagHs;

    string m_logs_path;
    string m_data_path;

    std::vector<vpImage<unsigned char> > m_desired_images;
    std::vector<vpImage<unsigned char> > m_current_images;
    std::vector<vpImage<unsigned char> > m_diff_images;


    int m_index_desired_image_fwd;
    int m_index_desired_image_bwd;
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

    bool m_vs_translation;
    bool m_pure_rotation;
    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    geometry_msgs::Twist m_velocity;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    vpDisplayX m_display_prev_desired_image;
    //    double m_current_error;
    //    double m_pred_error;
    double m_mu;
    double m_lambda;
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
    double m_center_image[2];
    double m_center_rotation[2];
    bool m_move_forward;
    bool m_move_backward;

    bool m_rotation_mode;

    VisualServoTools m_visual_servo_tools;
    bool m_flag;
    bool m_translation_mode;


};



/*
void move_backward(const sensor_msgs::ImageConstPtr& image)
{

    if (m_first_backward == 0)
    {
        initValues();
    }

    double t = vpTime::measureTimeMs();
    //
    // acquire image
    //
    this->m_current_image = visp_bridge::toVispImage(*image);

    this->multiply_image_by_mask(m_current_image, m_mask_image);
    vpDisplay::display(m_current_image);
    vpDisplay::flush(m_current_image);

    vpColVector e ;
    vpColVector v ; // camera velocity send to the robot
    stringstream str_diff, str_curr;
    //        int real_index;

    if ((this->m_index_desired_image_bwd >= 0) && (this->m_index_desired_image_bwd < m_desired_images.size()))
    {
        m_logfile<<"## Starting visual servo backward "<<m_index_desired_image_bwd<<endl;

        this->m_desired_image = this->m_desired_images[m_index_desired_image_bwd];
        this->multiply_image_by_mask(m_desired_image, m_mask_image);
        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);

        m_logfile<<"################################ Moving backward loading desired image n "<<m_index_desired_image_bwd<<endl;

        vpImageTools::imageDifference(m_current_image,m_desired_image, m_difference_image);
        vpDisplay::display(m_difference_image);
        vpDisplay::flush(m_difference_image);
        str_curr<<m_logs_path<<"current_images/current_image_"<<this->m_index_desired_image_bwd <<"_"<<m_iter<<".png";
        vpImageIo::write(m_current_image,str_curr.str());


        str_diff<<m_logs_path<<"diff_images/diff_image_"<<this->m_index_desired_image_bwd<<"_"<<m_iter<<".png";
        vpImageIo::write(m_difference_image,str_diff.str());
        this->m_sI.buildFrom(m_current_image) ;

        m_logfile<<"ici"<<endl;
        // ------------------------------------------------------
        // Control law
        //

        // ----------------------------------------------------------
        // Minimisation
        double lambdaGN = 5.0;

        // ----------------------------------------------------------
        int iter   = 1;
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations

        //            int iterGN = 20 ; // swicth to Gauss Newton after iterGN iterations

        double normeError = 0;

        // compute current error
        this->m_sI.error(*(*m_it_vec_sId_backward),this->m_error);

        // ---------- Levenberg Marquardt method --------------
        m_logfile<<"la"<<endl;

        if (m_iter == iterGN)
        {
            m_mu =  0.0001 ;
            m_lambda = lambdaGN;
        }

        vpMatrix H;

        if (m_current_error < this->m_pred_error && m_iter > iterGN)
        {
            m_mu /=2;
        }

        H = ((m_mu * (*m_it_diagHsd_backward)) +(*m_it_vec_Hsd_backward)).inverseByLU();

        //	compute the control law

        e = H * (*m_it_vec_Lsd_backward).t() *this->m_error;

        this->m_pred_error = this->m_current_error;
        normeError = (this->m_error.sumSquare());
        this->m_current_error = sqrt(normeError)/(this->m_height*this->m_width);
        m_logfile << "|e| "<< m_current_error<<std::endl ;
        m_logerror<<m_current_error<<endl;

        v = - m_lambda*e;

        t = vpTime::measureTimeMs() - t;
        m_logfile << "Time per frame "<<t<< std::endl;
        m_logfile << "velocity"<<v.t()<< std::endl;

        m_iter++;
    }

    m_logfile<<"after if  m_index_desired_image_bwd"<<m_index_desired_image_bwd<<endl;
    m_logfile<<"m_mu"<<m_mu<<endl;
    m_logfile<<"m_current_error"<<m_current_error<<endl;

    if ( (this->m_current_error < 0.2 && (m_index_desired_image_bwd >= 0)))
    {
        if (v[0]>0.0)
        {
            m_index_desired_image_bwd--;
            decrement();
            initValues();
            m_logfile<<"###################  move backward in if "<<m_index_desired_image_bwd<<endl;
        }
        else
        {
            if (v[0]>-0.1 && v[0]<0.0)
            {
                v[0] = -0.1;
            }
            m_logfile<<"v "<<v.t()<<endl;
            m_velocity.linear.x = v[0];
            m_velocity.angular.z = v[1];
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
        }

    }
    else if (m_index_desired_image_bwd == -1)
    {

        if ((m_index_desired_image_fwd == this->m_desired_images.size()+1))
        {
            m_logfile<<"##### reaching desired position forward"<<m_index_desired_image_fwd<<endl;
            m_logfile<<"##### reaching desired position backward"<<m_index_desired_image_bwd<<endl;

            m_first_forward  = 0;
            m_first_backward = 0;

            m_index_desired_image_bwd = this->m_desired_images.size() -1;
            m_index_desired_image_fwd = 1;
            initValues();
            initVisualPathFollowing();
            //                point_to_begin();
            //                point_to_end();
            //                decrement();
            //                this->initValues();
            this->m_nb_tours++;
            ros::spinOnce();

        }

        //            move_forward(image);
        //            this->m_first_forward++;

        //            m_velocity.linear.x = 0.0;
        //            m_velocity.angular.z = 0.0;
        //            m_logfile<<"velocity "<<m_velocity<<endl;
        //            m_ros_pub.publish(m_velocity);
        //            ros::spinOnce();
    }

    //        else if ((m_index_desired_image_bwd == -1) && (m_index_desired_image_fwd == this->m_desired_images.size()+1))
    //        {
    //           m_logfile<<"##### reaching desired position forward"<<m_index_desired_image_fwd<<endl;
    //           m_logfile<<"##### reaching desired position backward"<<m_index_desired_image_bwd<<endl;

    //            m_index_desired_image_bwd = this->m_desired_images.size() -1;
    //            m_index_desired_image_fwd = 1;
    //            point_to_begin();
    //            point_to_end();
    //            decrement();
    //            m_first_forward  = 0;
    //            m_first_backward = 0;
    //            this->initValues();
    //            this->m_nb_tours++;
    //        }



    else
    {
        m_velocity.linear.x = -0.1;
        m_velocity.angular.z = v[1];
        m_logfile<<"###################  move backward in else "<< v.t()<<endl;
        m_ros_pub.publish(m_velocity);
        ros::spinOnce();
    }

}
*/

//
// read matrices files
//

//        // read interaction matrices Hsd and diagHsd
//        for(int i=0;i<filenames_desired_images.size();i++)
//        {
//            vpMatrix Lsd;
//            stringstream  sstr_inter;
//            sstr_inter<<m_data_path<<"interaction/"<<filenames_interaction[i];
//            string filename = sstr_inter.str().c_str();
//            read_matrix_file(filename,Lsd);
////            m_logfile<<"###Lsd"<<endl<<Lsd<<endl;
//            m_logfile<<"reading Lsd"<<filename<<endl;
//            this->m_vec_Lsd.push_back(Lsd);
//        }

//        // read Hsd matrices Hsd and diagHsd
//        for(int i=0;i<filenames_desired_images.size();i++)
//        {
//            vpMatrix Hsd;
//            stringstream  sstr_hsd;
//            sstr_hsd<<m_data_path<<"Hsd/"<<filenames_Hsd[i];
//            string filename = sstr_hsd.str().c_str();
//            read_matrix_file(filename,Hsd);
//            m_logfile<<"reading Hsd"<<filename<<endl;

//            this->m_vec_Hsd.push_back(Hsd);
//        }

//        // read diagHsd matrices Hsd and diagHsd
//        for(int i=0;i<filenames_desired_images.size();i++)
//        {
//            vpMatrix diagHsd;
//            stringstream  sstr_diaghsd;
//            sstr_diaghsd<<m_data_path<<"diagHsd/"<<filenames_diagHsd[i];
//            string filename = sstr_diaghsd.str().c_str();
//            read_matrix_file(filename,diagHsd);
//            m_logfile<<"reading diagHsd"<<filename<<endl;
//            this->m_vec_diagHsd.push_back(diagHsd);
//        }


/*
if ( this->m_current_error < 0.02 && (m_index_desired_image_bwd >= 1))
{

    if (v[0]>0.0)
//            if (m_mu < 0.0001)
    {
        m_index_desired_image_bwd--;
        decrement();
        initValues();
        m_logfile<<"###################  move backward in if if "<<m_index_desired_image_bwd<<endl;
    }
    else
    {
        if (v[0] > 0.1 )
        {
            v[0] = -0.1;
        }
        //                m_logfile << "lambda = " << m_lambda << "  mu = " << m_mu <<endl;
        m_logfile<<"v "<<v.t()<<endl;
        m_velocity.linear.x = v[0];
        m_velocity.angular.z = v[1];
        m_ros_pub.publish(m_velocity);
        ros::spinOnce();
    }
}

else if (m_index_desired_image_bwd == 0)
{

//            m_index_desired_image_fwd --;
//            if (m_mu < 0.0001)
//            {
//                decrement();
//                initValues();
//                m_logfile<<"###################  desired image  "<<m_index_desired_image_fwd<<endl;
//            }

    m_velocity.linear.x = 0.0;
    m_velocity.angular.z = 0.0;
    m_ros_pub.publish(m_velocity);
    ros::spinOnce();
}

else
{
    if (fabs(v[0]) < 0.1 )
    {
        v[0] = -0.1;
    }

    //            m_logfile << "lambda = " << m_lambda << "  mu = " << m_mu <<endl;
    //            m_logfile<<"v "<<v.t()<<endl;

    m_velocity.linear.x = v[0];
    m_velocity.angular.z = v[1];
    m_ros_pub.publish(m_velocity);
    ros::spinOnce();
}
*/


