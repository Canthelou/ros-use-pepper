#include "VisualServoTools.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//openCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/Twist.h"
#include "visp_bridge/image.h"
#include <std_msgs/String.h>

//Libraries de projection
//#include <camera/CModelStereo.h>
#include <MIXEDVISION/CModelStereo.h>
#include <MIXEDVISION/CModelStereoXml.h>

//#include <CPoint.h>
#include <camera/CPoint.h>
#include <camera/CPerspective.h>

//#include <CPerspective.h>
//#include <CModelStereoXml.h>
#include <camera/CModel.h>

using namespace std;
using namespace cv;

class VisualPathFollowingWithKinect
{

public:

    VisualPathFollowingWithKinect()
        : m_it(m_nh)
    {
        m_sub_omni_image        = m_it.subscribe("/camera/image_raw", 1, &VisualPathFollowingWithKinect::imageCallbackuEye, this);
        m_sub_kinect_depth      = m_it.subscribe("/camera/depth_registered/image_raw", 1, &VisualPathFollowingWithKinect::imageCallbackdepth, this);
        m_sub_kinect_rgb        = m_it.subscribe("/camera/rgb/image_raw", 1, &VisualPathFollowingWithKinect::imageCallbackrgb, this);
    }

    ~VisualPathFollowingWithKinect()
    {

    }


    void imageCallbackuEye(const sensor_msgs::ImageConstPtr& msg)
    {
        //cv_bridge::CvImagePtr cv_ptr_omni;
        try
        {
            m_cv_ptr_omni = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat flipped_image;
        cv::flip(m_cv_ptr_omni->image, flipped_image, 0);
        cv::imshow("OpenCV viewer uEye RGB", flipped_image);
        cv::waitKey(3);
    }

    void imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr_depth;

        // Read the image
        try
        {
            cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        double minVal, maxVal;
        minMaxLoc(cv_ptr_depth->image, &minVal, &maxVal); //find minimum and maximum intensities
        //m_logfile<<"min\t"<<minVal<<"\tmaxval\t"<<maxVal<<endl;
        //Mat draw;
        cv::Mat depth_image;
        cv_ptr_depth->image.convertTo(depth_image, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

        cv::imshow("kinect depth image", depth_image);
        cvWaitKey(3);


        //***************************Projection kinect-->omni****************************************************

        //P (Majuscule): tout ce qui est en 3D
        //p (Miniscule): tout ce qui est en 2D

        //$^{c2}M_{c1}$ Matrice de transformation rigide: Kinect c1 -> Omni c2

        //Chargement du fichier xml de calibrage
        CModelStereo systeme(2);
        CModelStereoXml chargeSysteme("/home/yalj/MIXEDVISION_PO/MUI/bin64/calib.xml");
        chargeSysteme >> systeme;
        CPoint P;
        cv::Mat Imageprofomni(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, cv::DataType<float>::type);
        Scalar c(0);
        Imageprofomni=c;

        for(int v=0;v<cv_ptr_depth->image.rows;v++)
        {
            uint16_t* ptr_img_proj=cv_ptr_depth->image.ptr<uint16_t>(v);

            for(int u=0;u<cv_ptr_depth->image.cols;u++)
            {
                P.set_u(u);
                P.set_v(v);

                //Conversion du pixel en mètre
                systeme.cam[0]->pixelMeterConversion(P);

                //Projection perspective inverse dans le repère kinect (en métre)
                P.set_oX(*ptr_img_proj*P.get_x()*0.001);
                P.set_oY(*ptr_img_proj*P.get_y()*0.001);
                P.set_oZ(*ptr_img_proj*0.001);

                //Changement de repère $^{c2}M_{c1}$
                P.changeFrame(systeme.ciMc1[1]);

                //Projection du point XYZ dans le plan image normalisé (Dans le repère de la caméra omni)
                systeme.cam[1]->project3DImage(P);

                //Conversion du mètre en pixel (Dans l'image)
                systeme.cam[1]->meterPixelConversion(P);

                //Accéder au pixel (Ils sont réelles; il va falloir les arrondir)
                int u_omni=vpMath::round(P.get_u());
                int v_omni=vpMath::round(P.get_v());

                if(u_omni>=0 && u_omni<this->m_cv_ptr_omni->image.cols && v_omni>=0 && v_omni<this->m_cv_ptr_omni->image.rows)
                    Imageprofomni.at<float>(v_omni,u_omni)=sqrt(pow(P.get_X(),2)+pow(P.get_Y(),2)+pow(P.get_Z(),2));

                ptr_img_proj++;

            }
        }



        //****************************************************************************************

        //conversion float to 8UC
    //    cv::Mat image8U_new(cv_ptr_omni->image.rows, cv_ptr_omni->image.cols, CV_8UC1);
    //    for(int i=0;i<image8U_new.rows;i++)
    //    {
    //        float* ptr_img_32F_new = Imageprofomni.ptr<float>(i);
    //        char* ptr_img_8U_new = image8U_new.ptr<char>(i);

    //        for(int j=0;j<image8U_new.cols;j++)
    //        {
    //            *ptr_img_8U_new=255*(*ptr_img_32F_new)/3.5;
    //            ptr_img_8U_new++;
    //            ptr_img_32F_new++;
    //        }
    //    }

        //
        // better way to perform the conversoin from float to 8UC
        //

        cv::Mat image8U_new(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, CV_8UC1);

        minMaxLoc(Imageprofomni, &minVal, &maxVal); //find minimum and maximum intensities
        //m_logfile<<"min\t"<<minVal<<"\tmaxval\t"<<maxVal<<endl;

        Imageprofomni.convertTo(image8U_new, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));


        cv::Mat image8U_flip(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, CV_8UC1);

        //cv::imshow("image ", cv_ptr_omni->image);
        //Flip verticale de l'image
        //cv::flip(image8U_new, image8U_flip, 0);
        image8U_flip = image8U_new;
        //        cv::imshow("image flippé", image8U_flip);


        //*******************************blending two images*************************************
        // Paramètres de superposition
        double alpha = 0.1; double beta;
        beta = ( 1.0 - alpha );


        //        //Kinect+Omni **********************************************************
        Mat Image_Kinect_Omni;

        //Converstion de l'image kinect en 3 canaux
        cv::Mat image8U_flip_rgb(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, DataType<Vec3b>::type);
        cvtColor(image8U_flip, image8U_flip_rgb, CV_GRAY2RGB);

        cv::Mat flipped_image;
        cv::flip(this->m_cv_ptr_omni->image, flipped_image, 0);

        //Superposition des deux images
        addWeighted( flipped_image, alpha, image8U_flip_rgb, beta, 0.0, Image_Kinect_Omni);

        //cv::imshow("kinect", image8U_flip_rgb);
        cv::imshow( "image kinect omni", Image_Kinect_Omni);



        //	//Kinect (rgb+depth) **************************************************
        //	Mat Image_rgb_depth;
        //	if (cv_ptr_kinect_rgb != NULL ) addWeighted( cv_ptr_kinect_rgb->image, alpha, image8U, 0.5, 0.5, Image_rgb_depth);
        //	cv::imshow( "Superposition RGB+DEPTH Kinect", Image_rgb_depth );


    //    Enregister l'image ***************************************************
        char cc = cvWaitKey(33);
        if( cc == 32 ) //32 code ASCII "Espace" du clavier
        {
            ROS_INFO("Space key is pressed");
            vpImage<vpRGBa> Imvisp_Kinect_Omni;
            vpImageConvert::convert (Image_Kinect_Omni, Imvisp_Kinect_Omni);
            vpImageIo::write(Imvisp_Kinect_Omni,"/home/yalj/data/Imvisp_Kinect_Omni.png"); //par défaut dans ~/../catkin_ws/Imvisp_Kinect_Omni.jpg
        }

        //*****************************************************************************

        cvWaitKey(3);
    }

    void imageCallbackrgb(const sensor_msgs::ImageConstPtr& msg)
    {

        //cv_bridge::CvImagePtr cv_ptr_kinect_rgb;

        try
        {
            this->m_cv_ptr_kinect_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("OpenCV viewer Kinect RGB", this->m_cv_ptr_kinect_rgb->image);
        cvWaitKey(3);
    }

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_sub_omni_image;
    image_transport::Subscriber m_sub_kinect_depth;
    image_transport::Subscriber m_sub_kinect_rgb;


    ros::Publisher m_ros_pub;
    ros::Publisher m_ros_pub_im_file_name;
    ros::Subscriber m_sonar_sub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_desired_image_rotated;

    vpImage<unsigned char> m_mask_image;

    ofstream m_logfile;
    ofstream m_logerror;


    std::vector<string> m_vec_str_Ls;
    std::vector<string> m_vec_str_Hs;
    std::vector<string> m_vec_str_diagHs;

    string m_logs_path;
    string m_data_path;


    CFeatureLuminanceOmni m_sI ;


    vpColVector m_error ;

    int m_width;
    int m_height;
    int m_iter;
    geometry_msgs::Twist m_velocity;



    cv_bridge::CvImagePtr m_cv_ptr_omni;
    cv_bridge::CvImagePtr m_cv_ptr_kinect_rgb;

};
