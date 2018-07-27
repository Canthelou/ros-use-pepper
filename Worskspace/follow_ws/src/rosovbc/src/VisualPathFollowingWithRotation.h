#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <visp_bridge/image.h>
#include "VisualServoTools.h"
#include <visp/vpPlot.h>

using namespace std;

class VisualPathFollowingWithRotation
{

public:

    VisualPathFollowingWithRotation()
        : m_it(m_nh), m_plotter(2, 1000, 600, 1300, 0, "Photometric and velocity evolution")
    {
        m_omega_curr    = 0.0;
        m_omega_prev    = 0.0;

        m_width         = m_vs_tools.getWidth();
        m_height        = m_vs_tools.getHeight();
        m_nb_des_im     = 0;
        m_cam_param     = m_vs_tools.getCamParams();
        m_u0            = m_vs_tools.getU0()/2;
        m_v0            = m_vs_tools.getV0()/2;

        m_step          = 2.0;

        m_center[0]     = m_u0;
        m_center[1]     = m_v0;

        m_alpha         = 0.5;
        m_iter_pure_rotation = -1;

        m_b_combined_rotation_and_translation  = false;
        m_b_pure_rotation                      = true;
        m_nh.getParam("DI", m_DI);
        m_nh.getParam("DJ", m_DJ);
        m_nh.getParam("NBRI", m_NBRI);
        m_nh.getParam("NBRJ", m_NBRJ);

        m_DOF_VX            = true;
        m_DOF_VY            = true;
        m_DOF_VZ            = false;
        m_DOF_WX            = false;
        m_DOF_WY            = false;
        m_DOF_WZ            = false;


        m_curr_im         = vpImage<unsigned char>(m_height, m_width);
        m_des_im          = vpImage<unsigned char>(m_height, m_width);
        m_prev_des_im     = vpImage<unsigned char>(m_height, m_width);

        m_des_rot_im      = vpImage<unsigned char>(m_height, m_width);
        m_diff_im         = vpImage<unsigned char>(m_height, m_width);
        m_mask            = vpImage<unsigned char>(m_height, m_width);

        int px_margin_width = 65;
        int px_margin_height = 55;

        m_disp_curr.init(m_curr_im, px_margin_width, 0,"current image");
        m_disp_des.init(m_des_im, px_margin_width+m_width, 0,"desired image");
        m_disp_prev_des.init(m_prev_des_im, px_margin_width, px_margin_height+m_height,"prev desired image");
        m_disp_des_rot.init(m_des_rot_im, px_margin_width+m_width, px_margin_height+m_height, "desired rotated image");
        m_disp_diff.init(m_diff_im, px_margin_width+2*m_width, 0,"difference image");

        /// plotter

        // The first graphic contains 1 curve and the second graphic contains 1 curve also
        m_plotter.initGraph(0,1);
        m_plotter.initGraph(1,2);

        m_plotter.setLegend(0,0, "photometric error evolution");
        m_plotter.setLegend(1,0, "v_x evolution");
        m_plotter.setLegend(1,1, "v_z evolution");

        // The color of the curve in the first graphic is red
        m_plotter.setColor(0,0, vpColor::red);
        m_plotter.setColor(1,0, vpColor::blue);
        m_plotter.setColor(1,1, vpColor::darkGreen);


        string camera_topic, outputVelocityTopic;
        m_nh.getParam("logs",                   m_logs_path);
        m_nh.getParam("data",                   m_data_path);
        m_nh.getParam("cameraTopic",            camera_topic);
        m_nh.getParam("outputVelocityTopic",    outputVelocityTopic);
        m_nh.getParam("lambda",                 m_lambda);
        m_nh.getParam("mu",                     m_mu);

        cout<<"lambda\t"<<m_lambda<<"\tm_mu="<<m_mu<<endl;
        stringstream sstr_log;
        sstr_log<<m_logs_path<<"logfile_visual_path_following.txt";
        m_logfile.open(sstr_log.str().c_str());

        /// subscribe to camera topic
        // not sure about the 1 => TODO investigate the impact of different values on the performance of visual servo
        m_image_sub = m_it.subscribe(camera_topic, 1, &VisualPathFollowingWithRotation::imageCallback, this);

        /// advertize to robot topic
        // not sure about the 1 => TODO investigate the impact of different values on the performance of visual servo
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(outputVelocityTopic, 1);

        /// subscribe to robot topic
        m_ros_sub = m_nh.subscribe<geometry_msgs::Twist>(outputVelocityTopic, 1,
                                                         &VisualPathFollowingWithRotation::robotVelocityCallback, this);

        //        /// callback timer if no signal for 10 seconds then quit the program
        //        m_timer = m_nh.createTimer(ros::Duration(10.0), &VisualPathFollowingWithRotation::callbackTimer, this);

        initVisualPathFollowing();

        m_b_move_forward = true;
        m_b_move_backward = false;

        m_count = 0;
    }


    void initVisualPathFollowing()
    {
        /// load desired images
        stringstream str;
        str<<m_logs_path<<"desired_images/";
        string filename_prefix = "desired_image";
        std::vector<string> filenames_desired_images;

        m_vs_tools.load_directory_and_sort_files(str.str().c_str(), filename_prefix.c_str(), filenames_desired_images);

        m_vec_des_images = std::vector<vpImage<unsigned char> > (filenames_desired_images.size());
        /// read the mask
        read_mask();
        //
        // read desired images
        //
        for (unsigned int i = 0;  i< filenames_desired_images.size(); i++)
        {

            m_logfile<<"loading desired image\t"<<i<<endl;
            stringstream  sstr_des;
            string filename;

//            m_vec_des_images[i].init(this->m_height, m_width);
            sstr_des <<m_logs_path<<"desired_images/"<<filenames_desired_images[i];
            filename = sstr_des.str();
            vpImageIo::read(m_des_im, filename);
            m_vec_des_images[i] = m_des_im;

            /// build desired features
            m_sId = new CFeatureLuminanceOmni();
            m_sId->setInterpType(INTERPTYPE);

            m_sId->init(m_height, m_width,
                        m_DI, m_DJ, m_NBRI, m_NBRJ, PAS, &m_mask, m_vs_tools.getRHO(), REPTYPE, GRAPCALCTYPE) ;

            m_sId->set_DOF(m_DOF_VX, m_DOF_VY, m_DOF_VZ,
                           m_DOF_WX, m_DOF_WY, m_DOF_WZ);
            m_sId->buildFrom(m_des_im);
            m_sId->interaction(m_Lsd) ;

            m_vec_Lsd.push_back(m_Lsd);
            m_vec_sId.push_back(m_sId);
        }


        /// init values
        m_iter                  = -1;
        m_iter_visual_servo     = -1;
        m_nb_backward           = 0;
        m_index_desired_image   = 0;
        m_current_error         = DBL_MAX;
        m_prev_error            = 0.0;

        m_sI.init(m_height, m_width, m_DI, m_DJ, m_NBRI, m_NBRJ,
                  PAS, &m_mask, m_vs_tools.getRHO(), REPTYPE, GRAPCALCTYPE);
        m_sI.setCameraParameters(m_cam_param) ;
        m_sI.set_DOF(m_DOF_VX, m_DOF_VY, m_DOF_VZ,
                     m_DOF_WX, m_DOF_WY, m_DOF_WZ);
        m_sI.setInterpType(INTERPTYPE);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& im_ptr)
    {
        m_iter++;
        m_logfile<<"#########  m_iter\t"<<m_iter<<endl;

        vpImage<unsigned char> tmp_curr(m_height, m_width);

        tmp_curr = visp_bridge::toVispImage(*im_ptr);
        tmp_curr.halfSizeImage(m_curr_im);

        CImageTools::imageZN(m_curr_im, m_curr_im);

        multiply_image_by_mask(m_curr_im, m_curr_im);


        /// write image
        stringstream sstr_curr;
        sstr_curr<<m_logs_path<<"current_images/current_image_"<<setw(4)<<setfill('0')
                <<m_index_desired_image<<"_"<<setw(4)<<setfill('0')<<m_iter<<".png";
        vpImageIo::write(m_curr_im, sstr_curr.str().c_str());

        stringstream str_des_im;

        if (m_b_move_forward && m_iter >0)
        {
            m_iter_visual_servo++;
            display_current_image();

            /// consider the first desired features after reverting the vectors

            if(m_index_desired_image >= m_vec_des_images.size())
            {
                geometry_msgs::Twist velocity;
                velocity.linear.x = 0.0;
                velocity.angular.z = 0.0;
                m_ros_pub.publish(velocity);
                //                ros::shutdown();
            }
            else
            {
                m_des_im        = m_vec_des_images[m_index_desired_image];
                m_Lsd           = m_vec_Lsd[m_index_desired_image];
                m_sId           = m_vec_sId[m_index_desired_image];

                m_logfile<<"################## m_index_desired_image"<<m_index_desired_image<<endl;
                vpDisplay::display(m_des_im);
                vpDisplay::flush(m_des_im);

                vpImageTools::imageDifference(m_curr_im, m_des_im, m_diff_im);
                vpDisplay::display(m_diff_im);
                vpDisplay::flush(m_diff_im);


                /// perform pure rotation

                if (m_b_pure_rotation)
                {
                    m_iter_pure_rotation++;
                    m_logfile<<"pure rotation triggered"<<endl;
                    cout<<"pure rotation triggered"<<endl;

                    // compute theta the angle between desired and current
                    double theta_degrees = find_best_angle(m_des_im, m_curr_im, m_center, m_step, m_logfile);

                    double theta_radians = vpMath::rad(theta_degrees);
                    geometry_msgs::Twist velocity;

                    m_omega_prev = m_omega_curr;
                    m_omega_curr = -m_alpha*theta_radians;

                    m_logfile<<"m_omega_prev\t"<<m_omega_prev<<endl;
                    m_logfile<<"m_omega_curr\t"<<m_omega_curr<<endl;
                    int sign_product = vpMath::sign(m_omega_curr)*vpMath::sign(m_omega_prev);
                    m_logfile<<"signs \t"<<vpMath::sign(m_omega_curr)<<"\t"<<vpMath::sign(m_omega_prev)<<endl;
//                    if (fabs(omega)< 2e-3)

                    velocity.linear.x  = 0.0;
                    velocity.angular.z = m_omega_curr ;//+vpMath::sign(m_omega_curr)*0.05;
                    m_ros_pub.publish(velocity);


//                    if ( ((sign_product == -1) && (m_iter_pure_rotation > 0)) || (m_iter_pure_rotation > 10))
                    if (fabs(m_omega_curr) < 8e-2 )
                    {
                        geometry_msgs::Twist velocity;
                        velocity.linear.x  = 0.0;
                        velocity.angular.z = 0.0;
                        m_ros_pub.publish(velocity);
                        m_b_pure_rotation = false;
                        m_b_combined_rotation_and_translation = true;
                        m_iter_pure_rotation = -1;
                    }
                    else
                    {
                    }
                }

                // if stability of the system

                else if (m_b_combined_rotation_and_translation)
                {
                    m_iter_visual_servo++;
                    m_logfile<<"combined rotation and translation triggered"<<endl;
                    cout<<"combined rotation and translation triggered"<<endl;

                    /// perform an iteration of visual servoing using Gauß Newton for combined rotation and translation
//                    gauss_newton_vs_iteration(m_curr_im, m_sI, m_sId, m_Lsd,
//                                                   m_lambda, m_current_error, m_vp_velocity);

                    robust_gauss_newton_vs_iteration(m_curr_im, m_sI, m_sId, m_Lsd,
                                                     m_lambda, m_current_error, m_vp_velocity);

                    m_logfile<<"vs iteration done"<<endl;
                    plot_and_save_curves();

                    m_logfile<<"m_vp_velocity\t"<<m_vp_velocity<<endl;
                    m_logfile<<"m_current_error\t"<<m_current_error<<endl;

                    if (m_vp_velocity[0] < 0.1 && m_vp_velocity[0] > 0.0)
                    {
                        m_vp_velocity[0] = 0.2;
                    }
                    if (m_vp_velocity[0] < 0.0 )
                    {
                        m_index_desired_image++;
                        m_b_pure_rotation = true;
                        m_alpha = 1.0;
                    }

                    else
                    {
                        geometry_msgs::Twist velocity;
                        velocity.linear.x  = m_vp_velocity[0];
                        velocity.angular.z = 1.5*m_vp_velocity[1];
                        m_ros_pub.publish(velocity);
                        ros::spinOnce();
                    }
                }
            }
        }
    }

    void robust_gauss_newton_vs_iteration(vpImage<unsigned char>& curr_im, CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                          vpMatrix Lsd, double lambda, double& current_error, vpColVector& velocity)
    {
        /// declare robust visual servo
        vpRobust robust(0) ;
        robust.setThreshold(0.000) ;

        /// vector containing the weights generated by the M-estimation
        vpColVector w ;
        vpColVector error;
        /// compute feature for each captured image
        sI.buildFrom(curr_im);
        //        m_logfile<<"built feautures"<<endl;

        double normeError = 0.0;
        /// compute current error
        sI.error(*sId, error);
        //        m_logfile<<"error computed\t"<<error.size()<<endl;

        /// the vector of error weighted by the values generated by the M-estimation
        vpColVector weighted_error;
        w.resize(error.size());
        weighted_error.resize(error.size());
        w = 1;

        /// once the error computed, generate the weights
        robust.setIteration(0);
        robust.MEstimator(vpRobust::TUKEY, error, w);
        //        m_logfile<<"generated weights"<<endl;
        int rows = Lsd.getRows();
        //        cout<<"nb rows"<<rows<<endl;
        vpMatrix w_Lsd(rows, Lsd.getCols());

        /// use the weights to discriminate or not pixels of the interaction matrix
        for (int i=0; i<rows; i++)
        {
            weighted_error[i] = w[i]*error[i];
            w_Lsd[i][0] = Lsd[i][0]*w[i];
            w_Lsd[i][1] = Lsd[i][1]*w[i];
        }

        vpColVector e,v;

        ///	compute the new control law
        e = w_Lsd.pseudoInverse()*weighted_error;
        normeError = (weighted_error.sumSquare());
        current_error = sqrt(normeError)/(weighted_error.getRows());
        velocity = - lambda*e;
        return;

    }


    void gauss_newton_vs_iteration(vpImage<unsigned char>& curr_im, CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                   vpMatrix Lsd, double lambda, double& current_error, vpColVector& velocity)
    {
        vpColVector error;
        /// compute feature for each captured image
        sI.buildFrom(curr_im);
        m_logfile<<"current features builts"<<endl;
        double normeError = 0.0;
        /// compute current error
        sI.error(*sId, error);
        m_logfile<<"error computed"<<endl;

        vpColVector e,v;
        e = Lsd.pseudoInverse()*error;
        m_logfile<<"control law done"<<endl;

        normeError = (error.sumSquare());
        current_error = sqrt(normeError)/(error.getRows());
        velocity = - lambda*e;
        m_logfile<<"velocity computed"<<endl;

    }

    void perform_combined_rotation_and_translation()
    {

        geometry_msgs::Twist velocity;

        m_logfile<<"combined rotation and translation"<<endl;
        /// find best angle between current and desired image
        double theta = find_best_angle(m_des_im, m_curr_im, m_center, m_step, m_logfile);
        //                cout<<"theta\t"<<theta<<endl;
        double theta_radians = vpMath::rad(theta);

        /// rotate desired image by theta

        m_logfile<<"theta\t"<<theta<<endl;
        m_logfile<<"theta_radians"<<theta_radians<<endl;

        /// perform a pure rotation until theta is null

        double omega = -0.5*theta_radians;
        velocity.angular.z = omega;
        m_ros_pub.publish(velocity);

        /*
        rotate_image(m_des_im, m_center,theta, m_des_rot_im, m_logfile);
        vpDisplay::display(m_des_rot_im);
        vpDisplay::flush(m_des_rot_im);

        vpImageTools::imageDifference(m_curr_im, m_des_rot_im, m_diff_im);
        vpDisplay::display(m_diff_im);
        vpDisplay::flush(m_diff_im);


        /// build desired features on the rotated image
        build_and_save_desired_features();

        m_prev_error = m_current_error;

        perform_visual_servo_iteration(m_curr_im, m_sI, m_sId, m_Lsd,
                                              m_Hsd, m_diagHsd, m_lambda, m_mu,
                                              m_current_error, m_vp_velocity);


        cout<<"theta \t"<<theta<<endl;
        cout<<"m_iter \t"<<m_iter<<endl;
        cout<<"m_index_desired_image \t"<<m_index_desired_image<<endl;
        cout<<"m_vp_velocity \t"<<m_vp_velocity<<endl;

        delete m_sId;

        //                m_logfile<<"### m_Lsd"<<endl<<m_Lsd<<endl;
        m_logfile<<"velocity\t"<<m_vp_velocity<<endl;
        m_logfile<<"m_current_error\t"<<m_current_error<<endl;
        plot_and_save_curves();

        /// criterion for passing to next desired image
        //&& (m_vp_velocity[0]< 0.0)
        //|| m_iter> 40
//        if( (m_prev_error < m_current_error) )
//        {
//            m_iter = -1;
//            this->m_index_desired_image++;
//            m_logfile<<"######################### m_index_desired_image\t"<<m_index_desired_image<<endl;
//            ros::spinOnce();
//        }



        velocity.linear.x  = m_vp_velocity[0];
        velocity.angular.z = m_alpha*m_vp_velocity[1] ; // for pure translation
        m_ros_pub.publish(velocity);


//        if (m_vp_velocity[0] > 0.0)
//        {
//            geometry_msgs::Twist velocity;
//            velocity.linear.x  = m_vp_velocity[0];
//            velocity.angular.z = m_alpha*m_vp_velocity[1]; // for pure translation
//            m_ros_pub.publish(velocity);

//        }

        */

    }

    void plot_and_save_curves()
    {
        m_logfile<<"plot curves"<<endl;
        /// plot and save plotted data
        m_plotter.plot(0, 0, m_iter_visual_servo, m_current_error);

        stringstream sstr_err, sstr_vel;
        sstr_err<<m_logs_path<<"photometric_error.dat";
        m_plotter.saveData(0, sstr_err.str().c_str());

        m_plotter.plot(1, 0, m_iter_visual_servo, m_vp_velocity[0]);
        m_plotter.plot(1, 1, m_iter_visual_servo, m_vp_velocity[1]);
        sstr_vel<<m_logs_path<<"velocity.data";
        m_plotter.saveData(1, sstr_vel.str().c_str());
        m_logfile<<"end plot curves"<<endl;

    }

    /// saving an image in a regular rate (e.g. one image over 25, 50, 100)

    bool criterion_des_im_regular_rate()
    {
        if ((m_iter%30 == 0))
        {
            m_logfile<<"m_iter\t"<<m_iter<<endl;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool criterion_des_im_odometry()
    {
        /// TODO
        return false;
    }

    void save_desired_image()
    {
        //        m_des_im  = m_curr_im;

        m_vec_des_images.push_back(m_des_im);
    }

    void display_current_image()
    {
        vpImagePoint ip1;

        // Display in overlay a rectangle.
        // The position of the top left corner is 300, 200.
        // The width is 200. The height is 100.

        ip1.set_i( m_DI );
        ip1.set_j( m_DJ );
        vpDisplay::display(m_curr_im);
        vpDisplay::displayRectangle(m_curr_im, ip1, m_NBRI, m_NBRJ,vpColor::red, false, 3) ;
        vpDisplay::flush(m_curr_im);
    }

    void build_and_save_desired_features()
    {
        m_sId = new CFeatureLuminanceOmni();
        m_sId->setInterpType(INTERPTYPE);

        m_sId->init(m_height, m_width,
                    m_DI, m_DJ, m_NBRI, m_NBRJ, PAS, &m_mask, m_vs_tools.getRHO(), REPTYPE, GRAPCALCTYPE) ;

        m_sId->set_DOF(m_DOF_VX, m_DOF_VY, m_DOF_VZ,
                       m_DOF_WX, m_DOF_WY, m_DOF_WZ);
//        m_sId->buildFrom(m_des_rot_im);
        m_sId->buildFrom(m_des_im);

        m_sId->interaction(m_Lsd) ;

        cout<<"m_Lsd\t"<<m_Lsd<<endl;


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
        m_vec_Lsd.push_back(m_Lsd);
        m_vec_Hsd.push_back(m_Hsd);
        m_vec_diagHsd.push_back(m_diagHsd);
        m_vec_sId.push_back(m_sId);

        m_logfile<<"building features image i"<<endl;

    }


    void perform_robust_visual_servo_iteration(vpImage<unsigned char>& curr_im, CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId, // input
                                               vpMatrix& Lsd, vpMatrix& Hsd, vpMatrix& diagHsd, // input
                                               double lambda, double mu, // input
                                               double& current_error,  vpColVector& velocity) //output
    {

        /// declare robust visual servo
        vpRobust robust(0) ;
        robust.setThreshold(0.000) ;

        /// vector containing the weights generated by the M-estimation
        vpColVector w ;
        vpColVector error;
        /// compute feature for each captured image
        sI.buildFrom(curr_im);
        //        m_logfile<<"built feautures"<<endl;

        double normeError = 0.0;
        /// compute current error
        sI.error(*sId, error);
        //        m_logfile<<"error computed\t"<<error.size()<<endl;

        /// the vector of error weighted by the values generated by the M-estimation
        vpColVector weighted_error;
        w.resize(error.size());
        weighted_error.resize(error.size());
        w = 1;

        /// once the error computed, generate the weights
        robust.setIteration(0);
        robust.MEstimator(vpRobust::TUKEY, error, w);
        //        m_logfile<<"generated weights"<<endl;
        int rows = Lsd.getRows();
        //        cout<<"nb rows"<<rows<<endl;
        vpMatrix w_Lsd(rows, Lsd.getCols());

        /// use the weights to discriminate or not pixels of the interaction matrix
        for (int i=0; i<rows; i++)
        {
            weighted_error[i] = w[i]*error[i];
            w_Lsd[i][0] = Lsd[i][0]*w[i];
            w_Lsd[i][1] = Lsd[i][1]*w[i];
        }

        //        m_logfile<<"weighted interactin matrix"<<endl;
        //        vpMatrix H;
        vpColVector e,v;

        //        m_logfile<<"diagHsd"<<diagHsd<<"\t"<<Hsd<<"\t"<<endl;
        //        H = ((mu * (diagHsd)) +(Hsd)).inverseByLU();
        //        m_logfile<<"H"<<H<<endl;

        //        m_logfile<<"weighted error size"<<weighted_error.size()<<endl;
        //        m_logfile<<"w_Lsd size"<<w_Lsd.getRows()<<"\t"<<w_Lsd.getCols()<<endl;
        //        m_logfile<<"H size"<<H.getRows()<<"\t"<<H.getCols()<<endl;

        ///	compute the new control law
        //        e = H * ((w_Lsd).t() *weighted_error);

        e = w_Lsd.pseudoInverse()*weighted_error;

        //        m_logfile<<"computed control law"<<endl;
        normeError = (weighted_error.sumSquare());
        current_error = sqrt(normeError)/(weighted_error.getRows());
        velocity = - lambda*e;
        return;
    }

    void perform_visual_servo_iteration(vpImage<unsigned char>& curr_im, CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId, // input
                                        vpMatrix& Lsd, vpMatrix& Hsd, vpMatrix& diagHsd, // input
                                        double lambda, double mu, // input
                                        double& current_error,  vpColVector& velocity) //output
    {

        vpColVector error;
        /// compute feature for each captured image
        sI.buildFrom(curr_im);

        double normeError = 0.0;
        /// compute current error
        sI.error(*sId, error);

        m_logfile<<"error computed"<<endl;
//        vpMatrix H;
        vpColVector e,v;
//        H = ((mu * (diagHsd)) +(Hsd)).inverseByLU();

        //	compute the control law
//        e = H * (Lsd).t() *error;
        m_logfile<<"Lsd"<<Lsd<<endl;
        e = Lsd.pseudoInverse()*error;

        m_logfile<<"control law"<<endl;

        // Gauss Newton
        // e = m_Hsd.inverseByLU()* m_Lsd.t()*error;
        //        m_prev_error = m_current_error;
        normeError = (error.sumSquare());
        current_error = sqrt(normeError)/(error.getRows());
        velocity = - lambda*e;
        m_logfile<<"velocity computed"<<endl;
        return;
    }




    void write_data()
    {
        // save images in hard drive

        int n = m_vec_des_images.size();

        for(int i = 0; i<n; i++)
        {
            stringstream sstr_des_image;
            sstr_des_image<<m_logs_path<<"desired_images/desired_image_"<<std::setw(4)<<std::setfill('0')<<i<<".png";
            vpImageIo::write(m_vec_des_images[i], sstr_des_image.str().c_str());
        }

    }

    void display_data()
    {
        vpDisplay::display(m_curr_im);
        vpDisplay::flush(m_curr_im);

        vpDisplay::display(m_des_im);
        vpDisplay::flush(m_des_im);

        vpDisplay::display(m_diff_im);
        vpDisplay::flush(m_diff_im);

        vpDisplay::display(m_mask);
        vpDisplay::flush(m_mask);
    }

    void robotVelocityCallback(const geometry_msgs::TwistConstPtr& twist_vs)
    {
        m_listened_vel.linear.x = twist_vs->linear.x;
        m_listened_vel.angular.z = twist_vs->angular.z;
    }

    void read_mask()
    {
        m_logfile<<"reading mask"<<endl;
        stringstream sstream_msk;
        sstream_msk<<m_data_path<<"mask/mask_.png";

        string mask_filename = sstream_msk.str().c_str();
        vpImage<unsigned char> tmp_mask(m_height, m_width);
        vpImageIo::read(tmp_mask, mask_filename);
        tmp_mask.halfSizeImage(m_mask);
    }

    void multiply_image_by_mask(vpImage<unsigned char>& src_im, vpImage<unsigned char>& dst_im)
    {
        // check if the image and the mask have the same size
        assert((src_im.getHeight() == m_mask.getHeight()) && (src_im.getWidth() == m_mask.getWidth()));
        assert((dst_im.getHeight() == m_mask.getHeight()) && (dst_im.getWidth() == m_mask.getWidth()));

        unsigned char *pt_mask = m_mask.bitmap;
        unsigned char *pt_src = src_im.bitmap;
        unsigned char *pt_dst = dst_im.bitmap;

        int size = src_im.getWidth()*src_im.getHeight();

        for(int i =0; i<size; i++, pt_mask++, pt_src++, pt_dst++)
        {
            *pt_dst  = *pt_mask & *pt_src;
        }
    }

    void clean()
    {
        m_logfile.close();
    }

    void image_difference(const vpImage<unsigned char>& im1, const vpImage<unsigned char>& im2, vpImage<unsigned char>& diff_im)
    {
        int width =  im1.getWidth();
        int height = im1.getHeight();
        int size = width*height;
        assert((im2.getHeight() == height) && (im2.getWidth() == width));

        diff_im.resize(height, width);
        int diff = 0;
        unsigned char *pt_im1 = im1.bitmap;
        unsigned char *pt_im2 = im2.bitmap;
        unsigned char *pt_diff = diff_im.bitmap;

        for(int i = 0; i <size; i++, pt_im1++, pt_im2++, pt_diff++)
        {
            diff = *pt_im1 - *pt_im2;
            *pt_diff = (unsigned char) (vpMath::maximum(vpMath::minimum(diff, 255), 0));
        }

    }

    ~VisualPathFollowingWithRotation()
    {

        for(int i=0; i<m_vec_sId.size(); i++)
            delete m_vec_sId[i];
    }

    void reverse_vectors()
    {
        std::reverse(this->m_vec_des_images.begin(), this->m_vec_des_images.end());
        std::reverse(this->m_vec_Lsd.begin(), this->m_vec_Lsd.end());
        std::reverse(this->m_vec_Hsd.begin(), this->m_vec_Hsd.end());
        std::reverse(this->m_vec_diagHsd.begin(), this->m_vec_diagHsd.end());
        std::reverse(this->m_vec_sId.begin(), this->m_vec_sId.end());
    }


    // dst_image typically is the desired image src_image is the current image
    double find_best_angle( vpImage<unsigned char>& src_image_1,vpImage<unsigned char>& src_image_2,
                            const double rotation_center[2], const double step, ostream& logfile)
    {
        cv::Mat mat_img_1, mat_img_2;
        //convert to opencv
        vpImageConvert::convert(src_image_1,mat_img_1) ;
        vpImageConvert::convert(src_image_2,mat_img_2) ;

        cv::Mat mat_rotated;
        cv::Mat diff;
        double min = DBL_MAX;
        double min_value;

        for (double i= 0.0; i<360.0; i=i+step)
        {
            // rotate image
            rotate(mat_img_1, rotation_center, i, mat_rotated);
            // compute difference
            cv::absdiff(mat_img_2, mat_rotated, diff);
            cv::Scalar sc_sum_diff  = cv::sum(diff);
            double sum_dff  = sqrt(sc_sum_diff.val[0])/(src_image_1.getHeight()*src_image_1.getWidth());
            //            logfile<<i<<"\t"<<sum_dff<<endl;
            if (sum_dff < min)
            {
                min = sum_dff;
                min_value = i;
            }
        }
        //        logfile<<"min_value"<<min_value<<endl;

        if (min_value > 180.0)
        {
            min_value -= 360.0;
        }

        return min_value;
    }



    void rotate_image(vpImage<unsigned char>& input_image, const double center[2], const double angle,
                      vpImage<unsigned char>& output_image, ostream& logfile)
    {
        cv::Mat cv_input_image, cv_dest_image;
        vpImageConvert::convert(input_image, cv_input_image);
        rotate(cv_input_image, center, angle, cv_dest_image);
        vpImageConvert::convert(cv_dest_image, output_image);
    }

    void rotate(cv::Mat& src, const double center[2], const double angle, cv::Mat& dst)
    {
        int len = std::max(src.cols, src.rows);
        cv::Point2f pt(center[0], center[1]);
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
        cv::warpAffine(src, dst, r, cv::Size(len, len));
    }


private:


    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;


    vpDisplayX m_disp_curr;
    vpDisplayX m_disp_des;
    vpDisplayX m_disp_prev_des;

    vpDisplayX m_disp_diff;
    vpDisplayX m_disp_des_rot;

    ros::Publisher m_ros_pub;
    ros::Subscriber m_ros_sub;

    string m_logs_path;
    string m_data_path;
    ofstream m_logfile;

    vpImage<unsigned char> m_curr_im;
    vpImage<unsigned char> m_des_im;
    vpImage<unsigned char> m_prev_des_im;

    vpImage<unsigned char> m_des_rot_im;
    vpImage<unsigned char> m_diff_im;
    vpImage<unsigned char> m_mask;

    std::vector<vpImage<unsigned char> > m_vec_curr_images;
    std::vector<vpImage<unsigned char> > m_vec_des_images;
    std::vector<vpImage<unsigned char> > m_vec_diff_images;

    geometry_msgs::Twist m_listened_vel;

    bool m_b_move_backward;
    bool m_b_move_forward;
    VisualServoTools m_vs_tools;
    ros::Timer m_timer;
    int m_width;
    int m_height;
    int m_iter;

    vpMatrix m_Lsd, m_Hsd, m_diagHsd;
    CFeatureLuminanceOmni *m_sId;
    CFeatureLuminanceOmni m_sI;


    std::vector<vpMatrix> m_vec_Lsd;
    std::vector<vpMatrix> m_vec_Hsd;
    std::vector<vpMatrix> m_vec_diagHsd;
    std::vector<CFeatureLuminanceOmni* > m_vec_sId;

    CCameraOmniParameters m_cam_param;
    double m_lambda;
    double m_mu;
    double m_current_error;
    double m_prev_error;
    int m_nb_des_im;
    int m_index_desired_image;
    int m_nb_backward;
    int m_iter_visual_servo;
    vpPlot m_plotter;
    vpColVector m_vp_velocity;

    vpImage<unsigned char> m_des_im_backward ;
    CFeatureLuminanceOmni *m_sId_backward;
    vpMatrix m_Lsd_backward;
    vpMatrix m_Hsd_backward;
    vpMatrix m_diagHsd_backward;

    int m_DI, m_DJ, m_NBRI, m_NBRJ;

    bool m_DOF_VX, m_DOF_VY, m_DOF_VZ,
    m_DOF_WX, m_DOF_WY, m_DOF_WZ;

    double m_u0, m_v0;
    int m_count;

    double m_step;
    double m_center[2];
    bool m_b_combined_rotation_and_translation;
    bool m_b_pure_rotation;

    double m_omega_curr;
    double m_omega_prev;
    double m_alpha;
    int m_iter_pure_rotation;
};

/*
//                if((m_index_desired_image >= 1) &&  (m_index_desired_image <= m_vec_des_images.size()))
//                {
//                    m_prev_des_im        = m_vec_des_images[m_index_desired_image - 1];
//                    vpDisplay::display(m_prev_des_im);
//                    vpDisplay::flush(m_prev_des_im);

//                    /// compute angle between current desired image and previous desired image
//                    double theta_des = find_best_angle(m_des_im, m_prev_des_im, m_center, m_step, m_logfile);

//                    if(theta_des > 5)
//                    {
//                        m_b_pure_rotation = true;
//                        if (m_b_pure_rotation)
//                        {
//                            m_logfile<<"performing pure rotation"<<endl;
//                            /// perform pure rotation
//                            perform_pure_rotation(theta_des);
//                            /// compute
//                            ros::spinOnce();
//                        }

//                    }

//                    else if(m_b_combined_rotation_and_translation)
//                    {
//                        m_logfile<<"switching to combined rotation and translation"<<endl;

//                        /// combined rotation and translation
//                        perform_combined_rotation_and_translation();
//                        ros::spinOnce();
//                    }
//                }
//                else
//                {
*/
