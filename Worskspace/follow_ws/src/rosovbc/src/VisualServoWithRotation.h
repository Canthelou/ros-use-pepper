
#include "visp_bridge/image.h"
#include "VisualServoTools.h"
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

using namespace std;

class VisualServoWithRotation
{

public:

    VisualServoWithRotation()
        : m_it(m_nh), m_desired_features_first_rotation(NULL), m_angle_first(-400.0)
    {

        m_width                             = m_visual_servo_tools.getWidth();
        m_height                            = m_visual_servo_tools.getHeight();

        m_px                                = m_visual_servo_tools.getPx();
        m_py                                = m_visual_servo_tools.getPy();
        m_u0                                = m_visual_servo_tools.getU0();
        m_v0                                = m_visual_servo_tools.getV0();
        m_xi                                = m_visual_servo_tools.getXi();

        m_center[0]                         = m_u0;
        m_center[1]                         = m_v0;

        m_iter                              = -1;

        string cameraTopic, robotTopic;

        m_nh.param("cameraTopic", cameraTopic, string(""));
        m_nh.param("robotTopic", robotTopic, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path,string(""));

        m_nh.param("lambda_first_rot", m_lambda_first_rot, double(20.0));
        m_nh.param("mu_first_rot", m_mu_first_rot, double(0.001));

        m_nh.param("lambda_second_rot", m_lambda_second_rot, double(20.0));
        m_nh.param("mu_second_rot", m_mu_second_rot, double(0.001));

        m_nh.param("lambda_combined_rot_trans", m_lambda_combined_rot_trans, double(20.0));
        m_nh.param("mu_combined_rot_trans", m_mu_combined_rot_trans, double(0.001));

        m_nh.param("alpha", m_alpha, double(1.0));
        m_nh.param("beta", m_beta, double(1.0));
        m_nh.param("gamma", m_gamma, double(1.0));

        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        m_nh.getParam("robotTopic", robotTopic);

        m_nh.getParam("lambda_first_rot", m_lambda_first_rot);
        m_nh.getParam("mu_first_rot", m_mu_first_rot);

        m_nh.getParam("lambda_combined_rot_trans", m_lambda_combined_rot_trans);
        m_nh.getParam("mu_combined_rot_trans", m_mu_combined_rot_trans);

        m_nh.getParam("lambda_second_rot", m_lambda_second_rot);
        m_nh.getParam("mu_second_rot", m_mu_second_rot);

        m_nh.getParam("alpha", m_alpha);
        m_nh.getParam("beta", m_beta);
        m_nh.getParam("gamma", m_gamma);

        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        m_logfile<<"opening logfile"<<endl;
        stringstream str_error;
        str_error<<m_logs_path<<"error.mat";
        m_logerror.open(str_error.str().c_str());

        m_current_image.init(m_height, m_width);
        m_desired_image.init(m_height, m_width);
        m_diff_image.init(m_height, m_width);
        m_mask_image.init(m_height, m_width);
        m_rotated_desired_image.init(m_height, m_width);


        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
        m_display_diff_image.init(this->m_diff_image, 2*m_width+10, 0, "Difference");
        m_display_rotated_desired_image.init(m_rotated_desired_image, 2*m_width+10, 2*m_height+30, "Rotated desired");


        m_image_sub = m_it.subscribe(cameraTopic, 1, &VisualServoWithRotation::imageCallback, this);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1);
        m_mask_image.init(m_height, m_width);

        initVisualServoingWithRotation();
        
        m_b_first_rotation_mode = true;
        m_b_combined_rotation_and_translation = false;
        m_b_second_rotation_mode = false;

    }

    void initVisualServoingWithRotation()
    {
        m_logfile<<"init visual servor"<<endl;
        std::string filename_read_image, filename_image_mask;
        // read desired image
        stringstream sstream_desired;
        sstream_desired<<m_logs_path<<"desired_images/desired_image.png";
        filename_read_image = vpIoTools::path(sstream_desired.str().c_str());

        // read mask image
        stringstream sstream_mask;
        sstream_mask<<m_data_path<<"mask/mask.png";
        filename_image_mask  = vpIoTools::path(sstream_mask.str().c_str());

	//Caron 2017
        /*vpImage<unsigned char> tmp_desired(m_height, m_width);
        vpImageIo::read(tmp_desired, filename_read_image) ;
        tmp_desired.halfSizeImage(m_desired_image);
	*/
	vpImageIo::read(m_desired_image, filename_read_image) ;

	//Caron 2017
        /*
        vpImage<unsigned char> temp_mask(m_height, m_width);
        m_visual_servo_tools.read_mask(filename_image_mask, temp_mask);
        temp_mask.halfSizeImage(m_mask_image);
	*/
        m_visual_servo_tools.read_mask(filename_image_mask, m_mask_image);

        m_visual_servo_tools.multiply_image_by_mask(m_desired_image, m_mask_image);
        m_visual_servo_tools.set_mask(m_mask_image);

        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);

        m_vs_tools_first_rotation.set_mask(m_mask_image);

        m_vs_tools_combined_rotation_and_translation.set_mask(m_mask_image);
        m_vs_tools_second_rotation.set_mask(m_mask_image);

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {

        m_iter++;
        m_logfile<<"m_iter "<<m_iter<<endl;	


	//Caron 2017
        /*
        vpImage<unsigned char> temp_curr_image (m_height, m_width);
        temp_curr_image = visp_bridge::toVispImage(*image);
        temp_curr_image.halfSizeImage(m_current_image);
	*/
	m_current_image = visp_bridge::toVispImage(*image);

        this->m_vs_tools_first_rotation.multiply_image_by_mask(m_current_image, m_mask_image);

        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

	if(true)//m_iter > 10)
	{
//        stringstream current_images_path;
//        current_images_path<<m_logs_path<<"current_images/acquisition"<<std::setw(4) << std::setfill('0')<<m_iter<<".png";

//        vpImageIo::write(m_current_image, current_images_path.str().c_str());

//        return;

        int step_angle_degrees = 2;

        if (m_b_first_rotation_mode)
        {
            m_logfile<<"#### First rotation mode"<<endl;

            m_desired_features_first_rotation  = new CFeatureLuminanceOmni();
	    if(m_angle_first == -400.0) //default value
	    {
                m_angle_first = m_vs_tools_first_rotation.find_best_angle(m_desired_image, m_current_image, m_center, step_angle_degrees*0.5, m_logfile);
	    }
	    else
	    {
                /*double angle*/ m_angle_first = m_vs_tools_first_rotation.find_best_angle_within_range(m_desired_image, m_current_image, m_center,
                                                                     step_angle_degrees, m_angle_first - 15, m_angle_first + 15, m_logfile);
	    }
            m_vs_tools_first_rotation.rotate_my_image(m_desired_image, m_center, m_angle_first, m_rotated_desired_image);

            // rotate the desired image comparing to the current image
            m_vs_tools_first_rotation.init_visual_servo(m_current_features_first_rotation, false, true, false, false, false, false,
                                                        m_logfile);
            m_vs_tools_first_rotation.build_desired_feature(m_rotated_desired_image, false, true, false, false, false, false,
                                                            m_desired_features_first_rotation,
                                                            m_Lsd_first_rot, m_Hsd_first_rot, m_diagHsd_first_rot);

            vpDisplay::display(m_rotated_desired_image);
            vpDisplay::flush(m_rotated_desired_image);

            m_vec_first_rot_desired_images.push_back(m_rotated_desired_image);
            m_vec_first_rot_current_images.push_back(m_current_image);


            m_current_features_first_rotation.buildFrom(m_current_image);

            vpMatrix H;
            vpColVector v, e, error;
            // compute current error
            m_current_features_first_rotation.error(*m_desired_features_first_rotation, error);
            H = ((m_mu_first_rot * (m_diagHsd_first_rot)) +(m_Hsd_first_rot)).inverseByLU();
            e = H * m_Lsd_first_rot.t() * error;

            m_logfile<<"error\t"<<e<<endl;
            m_prev_vel = m_curr_vel;
            m_curr_vel = -m_lambda_first_rot*e;

            // critere d'arret
            if (m_iter> 2)
            {
                if( ((vpMath::sign(m_curr_vel[0])*vpMath::sign(m_prev_vel[0])) == -1) && (fabs(m_curr_vel[0]) < 0.08))
                {
                    m_b_first_rotation_mode = false;
                    m_b_combined_rotation_and_translation = true;
                    m_logfile<<endl;
                    m_logfile<<endl;

                    m_iter = -1;
                    m_velocity.angular.z = 0.0;
                    m_ros_pub.publish(m_velocity);
                }
		else
		{
		//code specifique pour fauteuil Insa
	    if(fabs(m_curr_vel[0]) > 0.4)
		m_curr_vel[0] = vpMath::sign(m_curr_vel[0])*0.4;
	    else if(fabs(m_curr_vel[0]) < 0.2)
		{
		double minVel;
		if(m_iter < 5)
			minVel = 0.8;
		else
			minVel = 0.2;
		m_curr_vel[0] = vpMath::sign(m_curr_vel[0])*minVel;
		}

            m_logfile<<"### velocity\t"<<m_curr_vel<<endl;
            m_logfile<<"### m_alpha\t"<<m_alpha<<endl;
            m_velocity.angular.z = m_alpha*m_curr_vel[0];
            m_ros_pub.publish(m_velocity);
		}

            }



            delete m_desired_features_first_rotation;
            //ros::spinOnce();
        }
        else if(m_b_combined_rotation_and_translation)
        {
            m_logfile<<"## Combined rotation and translation mode"<<endl;

            m_desired_features_combined_rot_trans  = new CFeatureLuminanceOmni();

	    /*
            double theta_degrees = m_vs_tools_combined_rotation_and_translation.find_best_angle(m_desired_image, m_current_image, m_center,
                                                                                                step_angle_degrees, m_logfile);
            m_vs_tools_combined_rotation_and_translation.rotate_my_image(m_desired_image, m_center, theta_degrees,
                                                                         m_rotated_desired_image);
	    */
	    //ATTENTION, G. CARON, 29/11/2017 : A tester !! | Si pb, elargir les bornes pour la premiere iteration d'AV combinant rotation et translation
	    m_angle_first = m_vs_tools_combined_rotation_and_translation.find_best_angle_within_range(m_desired_image, m_current_image, m_center,
                                                                     step_angle_degrees, m_angle_first - 15, m_angle_first + 15, m_logfile);
	    m_vs_tools_combined_rotation_and_translation.rotate_my_image(m_desired_image, m_center, m_angle_first,
                                                                         m_rotated_desired_image);
	    
            m_vs_tools_combined_rotation_and_translation.init_visual_servo(m_current_features_combined_rot_trans,
                                                                           true, true, false, false, false, false,
                                                                           m_logfile);
            m_vs_tools_combined_rotation_and_translation.build_desired_feature(m_rotated_desired_image,
                                                                               true, true, false, false, false, false,
                                                                               m_desired_features_combined_rot_trans,
                                                                               m_Lsd_combined_rot_trans, m_Hsd_combined_rot_trans, m_diagHsd_combined_rot_trans);

            vpDisplay::display(m_rotated_desired_image);
            vpDisplay::flush(m_rotated_desired_image);

            m_current_features_combined_rot_trans.buildFrom(m_current_image);


            vpImageTools::imageDifference(m_current_image, m_rotated_desired_image, m_diff_image);
            vpDisplay::display(m_diff_image);
            vpDisplay::flush(m_diff_image);

            m_vec_combined_rot_trans_desired_images.push_back(m_rotated_desired_image);
            m_vec_combined_rot_trans_diff_images.push_back(m_diff_image);
            m_vec_combined_rot_trans_current_images.push_back(m_current_image);

            vpMatrix H;
            vpColVector v, e, error;
            // compute current error
            m_current_features_combined_rot_trans.error(*m_desired_features_combined_rot_trans, error);
            H = ((m_mu_combined_rot_trans * (m_diagHsd_combined_rot_trans)) +(m_Hsd_combined_rot_trans)).inverseByLU();
            e = H * m_Lsd_combined_rot_trans.t() * error;

            m_prev_vel  = m_curr_vel;
            m_curr_vel = -m_lambda_combined_rot_trans*e;

            m_logfile<<"lambda "<<m_lambda_combined_rot_trans<<"\t mu "<<m_mu_combined_rot_trans<<endl;
            m_logfile<<"### iteration\t"<<m_iter<<"\t velo"<<m_curr_vel<<endl;
            m_logfile<<"### beta\t"<<m_beta<<endl;

            // critère d'arret
            if( (m_curr_vel[0] <0.0) && (m_iter > 45))
	    //if (fabs(m_curr_vel[0]) < 0.004 && (m_iter > 45))
            {
                m_velocity.linear.x = 0.0;
                m_velocity.angular.z = 0.0;
		m_iter = -1;
                m_ros_pub.publish(m_velocity);
                m_b_second_rotation_mode = true;
                m_b_combined_rotation_and_translation =false;
                m_logfile<<endl;
                m_logfile<<endl;
            }
	else
	{
	    if(fabs(m_curr_vel[0]) > 0.5)
		m_curr_vel[0] = 0.5*m_curr_vel[0]/fabs(m_curr_vel[0]);
	   else if(fabs(m_curr_vel[0]) < 0.2)
		{
		double minVel;
		/*if(m_iter < 10)
			minVel = 0.8;
		else*/
			minVel = 0.2;
		m_curr_vel[0] = vpMath::sign(m_curr_vel[0])*minVel;
		}

            m_velocity.linear.x = m_curr_vel[0];
            m_velocity.angular.z = m_beta*m_curr_vel[1];
            m_ros_pub.publish(m_velocity);
	}


            delete m_desired_features_combined_rot_trans;
            //ros::spinOnce();

        }
        else if(m_b_second_rotation_mode)
        {

            m_logfile<<"## Second rotation mode"<<endl;

	    /*            
	    double theta_degrees = m_vs_tools_second_rotation.find_best_angle(m_desired_image, m_current_image, m_center,
                                                                              step_angle_degrees, m_logfile);
	    */
	    //ATTENTION, G. CARON, 29/11/2017 : A tester !! | Si pb, elargir les bornes pour la premiere iteration d'AV combinant rotation et translation
	    m_angle_first = m_vs_tools_second_rotation.find_best_angle_within_range(m_desired_image, m_current_image, m_center,
                                                                     step_angle_degrees, m_angle_first - 15, m_angle_first + 15, m_logfile);

//            vpImageTools::imageDifference(m_current_image, m_desired_image, m_diff_image);

//            vpDisplay::display(m_diff_image);
//            vpDisplay::flush(m_diff_image);



            m_vec_second_rot_current_images.push_back(m_current_image);

            //double theta_radians = vpMath::rad(theta_degrees);
	    //ATTENTION, G. CARON, 29/11/2017 : A tester !! | Si pb, elargir les bornes pour la premiere iteration d'AV combinant rotation et translation
	    double theta_radians = vpMath::rad(m_angle_first);

            double omega = -m_gamma*theta_radians;


            if (fabs(omega)< 2e-3)
            {
                m_velocity.angular.z = 0.0;
                m_ros_pub.publish(m_velocity);
                this->m_b_second_rotation_mode = false;
                write_data();
            }
		else
		{
	    if(fabs(omega) > 0.5)
		omega = vpMath::sign(omega)*0.5;
	    else if(fabs(omega) < 0.2)
		{
		double minVel;
		if(m_iter < 5)
			minVel = 1.0;
		else
			minVel = 0.2;
		m_curr_vel[0] = vpMath::sign(m_curr_vel[0])*minVel;
		}

            m_velocity.angular.z = omega; //+ vpMath::sign(omega)*0.05;
            m_ros_pub.publish(m_velocity);
		}

           // ros::spinOnce();

      }

      }
	/*else
            ros::spinOnce();*/
    }



    void write_data()
    {

        int size_combined_rot_trans = this->m_vec_combined_rot_trans_current_images.size();
        for(int i= 0; i<size_combined_rot_trans; i++)
        {
            stringstream sstream_curr, sstream_des, sstream_diff;
            sstream_curr<<m_logs_path<<"current_images/current_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_des<<m_logs_path<<"desired_images/rotated_desired_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_diff<<m_logs_path<<"diff_images/diff_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";

            vpImageIo::write(m_vec_combined_rot_trans_current_images[i], sstream_curr.str().c_str());
            vpImageIo::write(m_vec_combined_rot_trans_desired_images[i], sstream_des.str().c_str());
            vpImageIo::write(m_vec_combined_rot_trans_diff_images[i], sstream_diff.str().c_str());

        }

        int size_first_rot = this->m_vec_first_rot_current_images.size();
        for(int i= 0; i<size_first_rot; i++)
        {
            stringstream sstream_curr, sstream_des;
            sstream_curr<<m_logs_path<<"current_images/current_image_first_rot_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_des<<m_logs_path<<"desired_images/rotated_desired_image_first_rot"<<std::setw(4) << std::setfill('0')<<i<<".png";

            vpImageIo::write(m_vec_first_rot_current_images[i], sstream_curr.str().c_str());
            vpImageIo::write(m_vec_first_rot_desired_images[i], sstream_des.str().c_str());
        }


        int size_second_rot = this->m_vec_second_rot_current_images.size();
        for(int i= 0; i<size_second_rot; i++)
        {
            stringstream sstream_curr;
            sstream_curr<<m_logs_path<<"current_images/current_image_second_rot_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            vpImageIo::write(m_vec_second_rot_current_images[i], sstream_curr.str().c_str());
        }

    }


    ~VisualServoWithRotation()
    {

    }




private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;
    ros::Subscriber m_ros_sub;


    vpMatrix m_Lsd_first_rot, m_Hsd_first_rot, m_diagHsd_first_rot;
    vpMatrix m_Lsd_second_rot, m_Hsd_second_rot, m_diagHsd_second_rot;
    vpMatrix m_Lsd_combined_rot_trans, m_Hsd_combined_rot_trans, m_diagHsd_combined_rot_trans;



    vpImage<unsigned char> m_mask_image;
    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_rotated_desired_image;
    vpImage<unsigned char> m_diff_image;


    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_diff_image;
    vpDisplayX m_display_rotated_desired_image;


    VisualServoTools m_visual_servo_tools;
    VisualServoTools m_vs_tools_first_rotation;
    VisualServoTools m_vs_tools_combined_rotation_and_translation;
    VisualServoTools m_vs_tools_second_rotation;


    CFeatureLuminanceOmni *m_desired_features_first_rotation;
    CFeatureLuminanceOmni m_current_features_first_rotation;

    CFeatureLuminanceOmni *m_desired_features_combined_rot_trans;
    CFeatureLuminanceOmni m_current_features_combined_rot_trans;

    CFeatureLuminanceOmni *m_desired_features_second_rotation;
    CFeatureLuminanceOmni m_current_features_second_rotation;

    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_center[2];
    int m_width;
    int m_height;
    string m_logs_path;
    string m_data_path;

    double m_lambda_first_rot;
    double m_mu_first_rot;

    double m_lambda_second_rot;
    double m_mu_second_rot;

    double m_lambda_combined_rot_trans;
    double m_mu_combined_rot_trans;

    double m_alpha, m_beta, m_gamma;

    double m_angle_first;

    ofstream m_logfile;
    ofstream m_logerror;

    bool m_b_first_rotation_mode;
    bool m_b_combined_rotation_and_translation;
    bool m_b_second_rotation_mode;
    geometry_msgs::Twist m_velocity;
    int m_iter;
    vpColVector m_prev_vel;
    vpColVector m_curr_vel;


    std::vector<vpImage<unsigned char> > m_vec_first_rot_desired_images;
    std::vector<vpImage<unsigned char> > m_vec_first_rot_current_images;

    std::vector<vpImage<unsigned char> > m_vec_second_rot_current_images;

    std::vector<vpImage<unsigned char> > m_vec_combined_rot_trans_desired_images;
    std::vector<vpImage<unsigned char> > m_vec_combined_rot_trans_current_images;
    std::vector<vpImage<unsigned char> > m_vec_combined_rot_trans_diff_images;


};
