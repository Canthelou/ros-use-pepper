#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <image_transport/image_transport.h>
#include "VisualServoTools.h"
#include <geometry_msgs/Twist.h>
#include <visp_bridge/image.h>
#include <fstream>

using namespace std;


class PathManagerEnhanced
{
public:

    PathManagerEnhanced()
        : m_it(m_nh)
    {

        m_width     =  m_vs_tools.getWidth();
        m_height     =  m_vs_tools.getHeight();
        m_iter      = -1;

        string cameraTopic, robotTopic;

        m_nh.param("cameraTopic", cameraTopic, string(""));
        m_nh.param("robotTopic", robotTopic, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path, string(""));

        m_nh.param("lambda", m_lambda, double(0.0));
        m_nh.param("mu", m_mu, double(0.0));
        m_nh.param("difference_treshold", m_diff_threshold, double(0.0));
        m_nh.param("ratio_threshold", m_ratio_threshold, double(0.0));
        m_nh.param("max_iterations", this->m_max_iters, int (0));

        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("robotTopic", robotTopic);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        m_nh.getParam("lambda", m_lambda);
        m_nh.getParam("mu", m_mu);
        m_nh.getParam("difference_treshold", m_diff_threshold);
        m_nh.getParam("ratio_threshold", m_ratio_threshold);
        m_nh.getParam("max_iterations", this->m_max_iters);

        m_current_image                 = vpImage<unsigned char>(m_height, m_width);
        m_desired_image                 = vpImage<unsigned char>(m_height, m_width);
        m_rotated_desired_image         = vpImage<unsigned char>(m_height, m_width);
        m_diff_image                    = vpImage<unsigned char>(m_height, m_width);

        stringstream sstream_log_file;
        sstream_log_file<<m_logs_path<<"logfile.txt";
        this->m_logfile.open(sstream_log_file.str().c_str());

        initPathManager();
        m_b_visual_memory_acquisition = true;
        m_b_path_follow_backward      = false;

        m_disp_current_image.init(m_current_image, 0, 0, "Current image");
        m_disp_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
        m_disp_rotated_desired_image.init(m_rotated_desired_image, 2*m_width+10, 2*m_height+30, "Rotated desired");
        m_disp_diff_image.init(m_diff_image, 2*m_width+10, 2*m_height+30, "Difference image");

        m_timer = m_nh.createTimer(ros::Duration(10.0), &PathManagerEnhanced::callbackTimer, this);
        m_image_sub = m_it.subscribe(cameraTopic, 1, &PathManagerEnhanced::imageCallback, this);
        m_ros_publisher = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1);

        m_nb_backward = 0;


    }

    void initPathManager()
    {
        // read mask image
        stringstream sstream_mask;
        sstream_mask<<m_data_path<<"mask/mask.png";

        vpImage<unsigned char> temp_mask(m_height, m_width);
        m_vs_tools.read_mask(sstream_mask.str().c_str(), temp_mask);
        temp_mask.halfSizeImage(m_mask_image);

        m_vs_tools.set_mask(m_mask_image);

        m_b_save_visual_features = true;

        m_vs_tools.init_visual_servo(m_sI, true, false, false, false, false, true);
        m_logfile<<"init path manager done"<<endl;


    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {

        m_iter++;
        m_logfile<<"##### iteration"<<m_iter<<endl;

        vpImage<unsigned char> temp_curr_image(m_height, m_width);

        temp_curr_image = visp_bridge::toVispImage(*image);
        temp_curr_image.halfSizeImage(m_current_image);

        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);


        if (m_b_visual_memory_acquisition)
        {
            m_logfile<<"in m_b_visual_memory_acquisition"<<endl;

            if (m_b_save_visual_features)
            {
                m_logfile<<"in if m_b_save_visual_features"<<endl;

                this->m_desired_image = m_current_image;
                sId = new CFeatureLuminanceOmni();

                m_vs_tools.build_desired_feature(m_desired_image,
                                                       true, false, false,
                                                       false, false, true, sId, m_Lsd, m_Hsd, m_diagHsd);

                m_vec_desired_images.push_back(m_desired_image);
                m_vec_sId.push_back(sId);
                m_vec_Lsd.push_back(m_Lsd);
                m_vec_Hsd.push_back(m_Hsd);
                m_vec_diagHsd.push_back(m_diagHsd);

                m_b_save_visual_features = false;
//                sId->staticClean();
//                delete sId;

            }
            else
            {
                vpImageTools::imageDifference(m_current_image, m_desired_image, this->m_diff_image);

                m_logfile<<"in else m_b_save_visual_features"<<endl;

                vpColVector error,e;
                // init visual servoing

                m_sI.buildFrom(m_current_image);
                m_logfile<<"after build from"<<endl;
                m_sI.error(*(sId), error);
                m_logfile<<"after error"<<endl;

                vpMatrix H;
                H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();
                e = H*m_Lsd.t()*error;

                if (m_iter >=0)
                {
                    m_prev_vel = m_curr_vel;
                }
                m_curr_vel = - m_lambda*e;

                m_logfile<<"generating velocities"<<endl;


                if (m_iter == 0)
                {
                    this->m_first_it_vel = m_curr_vel;
                }


                if (m_iter >=0)
                {
                    m_logfile<<"m_curr_vel[0]"<<m_curr_vel[0]<<endl;
                    m_logfile<<"m_first_it_vel[0]"<<m_first_it_vel[0]<<endl;
                    m_logfile<<"m_prev_vel[0]"<<m_prev_vel[0]<<endl;


//                    if (((fabs(this->m_curr_vel[0]/this->m_first_it_vel[0])> m_ratio_threshold) &&
//                         (fabs(m_prev_vel[0] -  m_curr_vel[0]) <m_diff_threshold))
//                            || m_iter == m_max_iters)
                    {
                        m_b_save_visual_features = true;
                        m_iter = -1;

                    }

                }


                m_velocity.linear.x   = m_curr_vel[0];
                m_velocity.angular.z  = m_curr_vel[1];

                m_ros_publisher.publish(m_velocity);
                ros::spinOnce();

            }

        }

        else if (m_b_path_follow_backward)
        {



        }
    }


    void callbackTimer(const ros::TimerEvent&)
        {

            ROS_INFO("Callback 1 triggered");

            m_logfile<<"we havent received any velocity "<<endl;
            if ((m_listened_velocity.linear.x == 0.0) && (m_listened_velocity.angular.z == 0.0) && (m_iter !=-1))
            {
                m_logfile<<"writing data"<<endl;

                write_data();
                m_logfile<<"NO received velocity for during 10 seconds"<<endl;
                if (m_nb_backward == 0)
                {
                    m_logfile<<"speed is null"<<endl;

                    //
                    // reverse order of vectors
                    //
                    m_logfile<<"#### reverting vectors"<<endl;
                    reverse(m_vec_desired_images.begin(), m_vec_desired_images.end());
                    reverse(m_vec_Lsd.begin(), m_vec_Lsd.end());
                    reverse(m_vec_Hsd.begin(), m_vec_Hsd.end());
                    reverse(m_vec_diagHsd.begin(), m_vec_diagHsd.end());
                    reverse(m_vec_sId.begin(), m_vec_sId.end());
                    m_iter = -1;
                    m_b_save_visual_features  = false;
                    m_b_path_follow_backward = true;
                }
                m_nb_backward++;
            }
            else
            {
                m_logfile<<"m_listened_velocity\t"<<m_listened_velocity<<endl;
            }
        }

    void robotVelocityCallback(const geometry_msgs::TwistConstPtr& twist_vs)
    {
        m_listened_velocity.linear.x = twist_vs->linear.x;
        m_listened_velocity.angular.z = twist_vs->angular.z;
    }


    void write_data()
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
            m_vs_tools.write_matrices(m_vec_Lsd[i], m_vec_Hsd[i], m_vec_diagHsd[i], i, m_data_path);
        }
    }

    ~PathManagerEnhanced()
    {

    }

private:

    bool m_b_path_follow_backward;
    bool m_b_visual_memory_acquisition;

    ros::Publisher m_ros_publisher;
    ros::Subscriber m_ros_subscriber;
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    string m_logs_path;
    string m_data_path;

    unsigned int m_height;
    unsigned int m_width;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_rotated_desired_image;
    vpImage<unsigned char> m_diff_image;
    vpImage<unsigned char> m_mask_image;

    std::vector<vpImage<unsigned char> > m_vec_desired_images;

    vpDisplayX m_disp_current_image;
    vpDisplayX m_disp_diff_image;
    vpDisplayX m_disp_desired_image;
    vpDisplayX m_disp_rotated_desired_image;
    VisualServoTools m_vs_tools;
    int m_iter;
    ofstream m_logfile;

    int m_b_save_visual_features;

    std::vector<CFeatureLuminanceOmni *>m_vec_sId;
    std::vector<vpMatrix> m_vec_Lsd;
    std::vector<vpMatrix> m_vec_diagHsd;
    std::vector<vpMatrix> m_vec_Hsd;

    ros::Timer m_timer;

    CFeatureLuminanceOmni *sId ;
    CFeatureLuminanceOmni m_sI;
    vpMatrix m_Lsd, m_Hsd, m_diagHsd;
    double m_lambda, m_mu;
    vpColVector m_curr_vel, m_prev_vel, m_first_it_vel;
    geometry_msgs::Twist m_velocity;
    geometry_msgs::Twist m_listened_velocity;

    int m_max_iters;
    double m_diff_threshold, m_ratio_threshold;
    int m_nb_backward;
};
