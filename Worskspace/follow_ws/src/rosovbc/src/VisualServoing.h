#include "VisualServoTools.h"
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <image_transport/image_transport.h>
#include "visp_bridge/image.h"
#include <tf/transform_broadcaster.h>
//Joint
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <sensor_msgs/JointState.h>

//static const char WINDOW[] = "Image window";

using namespace std;

class VisualServoing
{


public:

    VisualServoing()
        : m_it(m_nh)
    {
        m_desired_features                  = new CFeatureLuminanceOmni();
        m_width                             = m_visual_servo_tools.getWidth();
        m_height                            = m_visual_servo_tools.getHeight();

        m_px                                = m_visual_servo_tools.getPx();
        m_py                                = m_visual_servo_tools.getPy();
        m_u0                                = m_visual_servo_tools.getU0();
        m_v0                                = m_visual_servo_tools.getV0();
        m_xi                                = m_visual_servo_tools.getXi();

        m_center[0]                         = m_u0;
        m_center[1]                         = m_v0;

        string cameraTopic, robotTopic, visualServoMovements, poseTopic;

        m_nh.param("cameraTopic", cameraTopic, string(""));
        //m_nh.param("robotTopic", robotTopic, string(""));
        //m_nh.param("poseTopic", poseTopic, string(""));
        m_nh.param("visualServoMovements", visualServoMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));
        m_nh.param("data", m_data_path,string(""));
        //m_nh.param("error_threshold",m_threshold_error, double(0.0));
        m_nh.param("lambda", m_lambda, double(10.0));
        m_nh.param("mu", m_mu, double(10.0));


        //m_nh.getParam("poseTopic", poseTopic);
        m_nh.getParam("cameraTopic", cameraTopic);
        m_nh.getParam("visualServoMovements", visualServoMovements);
        m_nh.getParam("logs", m_logs_path);
        m_nh.getParam("data", m_data_path);
        //m_nh.getParam("error_threshold",m_threshold_error);
        //m_nh.getParam("robotTopic", robotTopic);
        m_nh.getParam("lambda", m_lambda);
        m_nh.getParam("mu", m_mu);

        this->initValues();
        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());

        m_logfile<<"opening logfile"<<endl;
        stringstream str_error;
        str_error<<m_logs_path<<"error.mat";
        m_logerror.open(str_error.str().c_str());

        m_current_image.init(m_height, m_width);
        m_desired_image.init(m_height, m_width);
        m_difference_image.init(m_height, m_width);

        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
        m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");


        m_mask_image.init(m_height, m_width);
        
        m_image_sub = m_it.subscribe(cameraTopic, 1, &VisualServoing::imageCallback, this);
        
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1);
	
        //Vivien 2018
        //Add pub and sub for joint on pepper, undo the camera movement
	m_ros_jointStates = m_nh.subscribe("/joint_states", 1, &VisualServoing::jointCallback, this);
        m_ros_jointAngles = m_nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);
	
        initVisualServoing();
    }

    ~VisualServoing()
    {
        delete m_desired_features;
        m_logfile.close();

    }

    void initValues()
    {
        this->m_current_error = 9999;
        m_iter      = 0;
    }

    void initVisualServoing()
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
        //        m_current_features = new CFeatureLuminanceOmni();

//        m_lambda  = 20.0;
//        m_mu      = 0.001;
        //        m_visual_servo_tools.set_lambda(105.0);
        //        m_visual_servo_tools.set_mu(0.001);

        //        m_visual_servo_tools.init_params_with_MEstimator(m_logfile);
        //        m_visual_servo_tools.initialize();
        
        initVisualServo();
    }

    void initVisualServo()
    {
        m_visual_servo_tools.init_visual_servo(m_current_features, false, false, true, false, true, false);
        m_visual_servo_tools.build_desired_feature(m_desired_image, false, false, true, false, true, false, 
                                                   m_desired_features, m_Lsd, m_Hsd, m_diagHsd);


    }

    //Vivien 2018
    //Subscribe
    //Callback of the joint_states subscriber
    //Contre mouvement !
    void jointCallback(const sensor_msgs::JointState& joint)
    {
                /*naoqi_bridge_msgs::JointAnglesWithSpeed pub_joint;
                float averageHeadPitch = 0.60, averageHeadYaw = 0.0;
                pub_joint.speed = 0.1;
                pub_joint.joint_names.push_back("RShoulderPitch");
		pub_joint.joint_names.push_back("LShoulderPitch");
                pub_joint.joint_names.push_back("RElbowYaw");
		pub_joint.joint_names.push_back("LElbowYaw");
		pub_joint.joint_names.push_back("RHand");
		pub_joint.joint_names.push_back("LHand");
		pub_joint.joint_names.push_back("RShoulderRoll");
		pub_joint.joint_names.push_back("LShoulderRoll");
		pub_joint.joint_names.push_back("RElbowRoll");
		pub_joint.joint_names.push_back("LElbowRoll");
		pub_joint.joint_names.push_back("RWristYaw");
		pub_joint.joint_names.push_back("LWristYaw");
		//pub_joint.joint_names.push_back("HipRoll");
		//pub_joint.joint_names.push_back("HipPitch");
		//pub_joint.joint_names.push_back("KneePitch");
		pub_joint.joint_angles.push_back(1.75);
    	        pub_joint.joint_angles.push_back(1.75);
		pub_joint.joint_angles.push_back(1.5);
    	        pub_joint.joint_angles.push_back(-1.5);
		pub_joint.joint_angles.push_back(0.75);
    	        pub_joint.joint_angles.push_back(0.75);
                for(int i=0;i<6;i++)
                        pub_joint.joint_angles.push_back(0);

                //Check name and position of Pepper Head joint
                if(joint.name[0] == "HeadYaw" && joint.name[1] == "HeadPitch"){
                        if(joint.position[0] != averageHeadYaw){
                                //Add HeadYaw to pub_joint
                                pub_joint.joint_names.push_back("HeadYaw");
                                pub_joint.joint_angles.push_back(averageHeadYaw);
                        }
                        if(joint.position[0] < (averageHeadYaw-0.01) || joint.position[0] > (averageHeadYaw+0.01)){
                                //Add HeadPitch to pub_joint
                                pub_joint.joint_names.push_back("HeadPitch");
                                pub_joint.joint_angles.push_back(averageHeadPitch);
                        }
                }
                m_ros_jointAngles.publish(pub_joint);
                m_logfile<<"join pub"<<endl;	*/
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {
        m_iter++;
        m_logfile<<"############################### m_iter"<<m_iter<<endl;

        double t = vpTime::measureTimeMs();
        //
        // acquire image
        //

	//Caron 2017
        /*
        vpImage<unsigned char> tmp_image(m_height, m_width);
        tmp_image = visp_bridge::toVispImage(*image);
        tmp_image.halfSizeImage(m_current_image);
	*/
        m_current_image = visp_bridge::toVispImage(*image);
        

        //        this->m_current_image = visp_bridge::toVispImage(*image);
        m_visual_servo_tools.multiply_image_by_mask(m_current_image, m_mask_image);
        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);

        //
        // first mode rotate robot until finding the correct angle
        //
        vpColVector v, e;
        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);

        vpImageTools::imageDifference(this->m_current_image,this->m_desired_image, this->m_difference_image) ;
        vpDisplay::display(this->m_difference_image);
        vpDisplay::flush(this->m_difference_image);

        m_current_features.buildFrom(m_current_image);
	
	//Vivien 2018
        //m_vec_combined_rot_trans_diff_images.push_back(m_difference_image);
        //m_vec_combined_rot_trans_current_images.push_back(m_current_image);

        vpColVector error;

//        vpRobust robust(0);
//        robust.setThreshold(0.0);
//        vpColVector w;
//        vpColVector weighted_error;

        // ----------------------------------------------------------
        // Minimisation
        double lambdaGN = 4.0;
        // ----------------------------------------------------------
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations
        double normeError = 0;
        // compute current error

        m_current_features.error(*(m_desired_features), error);

//        w.resize(error.getRows());
//        weighted_error.resize(error.getRows());
//        w =1;
//        robust.MEstimator(vpRobust::TUKEY, error, w);

        // ---------- Levenberg Marquardt method --------------

        if (this->m_iter == iterGN)
        {
            m_logfile<<"m_iter"<<m_iter<<endl;
            this->m_mu =  1e-6 ;
            //m_lambda = lambdaGN;
        }


        vpMatrix H;

        H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();
//        vpMatrix interaction_matrix((m_Lsd).getRows(),(m_Lsd).getCols());
//        int nb_total_points = (m_Lsd).getRows();
//        for (int i=0; i<nb_total_points; i++)
//        {
//            weighted_error[i] = w[i]*error[i];
//            interaction_matrix[i][0] = m_Lsd[i][0]*w[i];
//            interaction_matrix[i][1] = m_Lsd[i][1]*w[i];
//        }

        //	compute the control law
//        e = H * interaction_matrix.t() * weighted_error;

        e = H*m_Lsd.t()*error;
        //        m_prev_error = m_current_error;
//        normeError = weighted_error.sumSquare();
        normeError = error.sumSquare();

        m_current_error = sqrt(normeError)/(m_current_image.getHeight()*m_current_image.getWidth());
        //        logfile << "### |e|\t"<< m_current_error<<"\tpred_error\t"<< m_prev_error<<std::endl ;
        m_logfile<<"lambda\t"<<m_lambda<<"\tmu\t"<<m_mu<<endl;
        v = -m_lambda*e;
        m_logfile<<"v\t"<<v<<endl;

	/*if( (fabs(v[0]) > 1e2) || (fabs(v[1]) > 1e2) )
	{
	*/
        	m_logfile<<"### velocity is not null"<<v<<endl;
        	m_velocity.linear.x = v[0];
	        m_velocity.angular.z = -v[1];
        	m_ros_pub.publish(m_velocity);
		//12/12/2017: G. Caron, modif : spinOnce inutile (voire mauvais) quand on a spin dans le main
        	//ros::spinOnce();
	/*}
	else
	{
		m_velocity.linear.x = m_velocity.angular.z = 0.0;
        	m_ros_pub.publish(m_velocity);
        	//ros::spinOnce();
		ros::shutdown();
	}
	*/
	
	//Vivien 2018
	write_data();
    }


    //Vivien 2018
    //Store current and diff image on the computer
    //lines from the node visualServoWitheRotation.h
    void write_data()
    {

        int size_combined_rot_trans = this->m_vec_combined_rot_trans_current_images.size();
        for(int i= 0; i<size_combined_rot_trans; i++)
        {
            stringstream sstream_curr, sstream_des, sstream_diff;
            sstream_curr<<m_logs_path<<"current_images/current_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";
            sstream_diff<<m_logs_path<<"diff_images/diff_image_combined_trans_and_rot_mode_"<<std::setw(4) << std::setfill('0')<<i<<".png";

            vpImageIo::write(m_vec_combined_rot_trans_current_images[i], sstream_curr.str().c_str());
            vpImageIo::write(m_vec_combined_rot_trans_diff_images[i], sstream_diff.str().c_str());
        }
    }





private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub, m_ros_jointAngles;
    ros::Subscriber m_ros_sub, m_ros_jointStates;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_difference_image;

    vpImage<unsigned char> m_mask_image;

    ofstream m_logfile;
    ofstream m_logerror;

    string m_logs_path;
    string m_data_path;

    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    geometry_msgs::Twist m_velocity;

    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;

    double m_current_error;
    double m_pred_error;
    double m_mu;
    double m_lambda;


    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_center[2];
    VisualServoTools m_visual_servo_tools;

    CFeatureLuminanceOmni *m_desired_features;
    CFeatureLuminanceOmni m_current_features;

    double m_rho;
    vpMatrix m_Lsd, m_diagHsd, m_Hsd;
    
    //Vivien 2018
    std::vector<vpImage<unsigned char> > m_vec_combined_rot_trans_current_images;
    std::vector<vpImage<unsigned char> > m_vec_combined_rot_trans_diff_images;

};

