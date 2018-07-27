#include "VisualServoingWithKinect.h"

VisualServoingWithKinect::VisualServoingWithKinect()
    : m_it(m_nh)
{

    m_nh.getParam("logs", m_logs_path);
    stringstream sstream;
    sstream<<m_logs_path<<"logfile_kinect_visual_servo.txt";
    m_logfile.open(sstream.str().c_str());




    m_desired_features                  = new CFeatureLuminanceOmni();
    m_width                             = m_vs_tools.getWidth();
    m_height                            = m_vs_tools.getHeight();

    m_px                                = m_vs_tools.getPx();
    m_py                                = m_vs_tools.getPy();
    m_u0                                = m_vs_tools.getU0();
    m_v0                                = m_vs_tools.getV0();
    m_xi                                = m_vs_tools.getXi();
    m_rho                               = m_vs_tools.getRHO();

    m_center[0]                         = m_u0;
    m_center[1]                         = m_v0;

    m_cam_param                         = m_vs_tools.getCamParams();

    string cameraTopic, visualServoMovements, poseTopic;

    m_nh.getParam("cameraTopic", cameraTopic);
    m_nh.getParam("visualServoMovements", visualServoMovements);
    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("data", m_data_path);
    m_nh.getParam("error_threshold",m_threshold_error);
    m_nh.getParam("lambda", m_lambda);
    m_nh.getParam("mu", m_mu);

    this->initValues();


    m_logfile<<"opening logfile"<<endl;

      stringstream str_error;
    str_error<<m_logs_path<<"error.mat";
    m_logerror.open(str_error.str().c_str());

    m_current_image.init(m_height, m_width);
    m_desired_image.init(m_height, m_width);
    m_difference_image.init(m_height, m_width);
    m_depth_image.init(m_height, m_width);
    m_desired_depth_image.init(m_height, m_width);

    m_display_current_image.init(m_current_image, 0, 0, "Current image");
    m_display_desired_image.init(m_desired_image, m_width+10, 0, "Desired image");
    m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");

    m_mask_image.init(m_height, m_width);

    initVisualServoing();



    m_sub_omni_image        = m_it.subscribe("/camera/image_raw", 1, &VisualServoingWithKinect::imageCallbackuEye, this);
    m_sub_kinect_depth      = m_it.subscribe("/camera/depth_registered/image_raw", 1,
                                             &VisualServoingWithKinect::imageCallbackdepth, this);
    m_sub_kinect_rgb        = m_it.subscribe("/camera/rgb/image_raw", 1, &VisualServoingWithKinect::imageCallbackrgb, this);

    m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(visualServoMovements, 1);


}



VisualServoingWithKinect::~VisualServoingWithKinect()
{
    delete m_desired_features;
}


void VisualServoingWithKinect::initValues()
{
    this->m_current_error = 9999;
    m_iter      = 0;
}


void VisualServoingWithKinect::initVisualServoing()
{
    m_logfile<<"init visual servor"<<endl;
    std::string filename_read_image, filename_image_mask, filename_depth_image;
    // read desired image
    stringstream sstream_desired;
    sstream_desired<<m_logs_path<<"desired_images/desired_image.png";

    m_logfile<<"desired image"<<sstream_desired.str().c_str()<<endl;

    filename_read_image = vpIoTools::path(sstream_desired.str().c_str());
    vpImageIo::read(m_desired_image, filename_read_image) ;
    cv::Mat cv_desired_image, flipped_image;
    vpImageConvert::convert(m_desired_image, cv_desired_image);

    cv::flip(cv_desired_image, flipped_image, 0);
    vpImageConvert::convert(flipped_image, m_desired_image);


    // read mask image
    stringstream sstream_mask;
    sstream_mask<<m_data_path<<"mask/mask.png";
    filename_image_mask  = vpIoTools::path(sstream_mask.str().c_str());
    vpImageIo::read(m_mask_image, filename_image_mask) ;

    //
    // read depth image
    //
    stringstream sstream_depth_image;
    sstream_depth_image<<m_data_path<<"desired_images/desired_image_depth.png";
    filename_depth_image  = vpIoTools::path(sstream_mask.str().c_str());
    vpImage<unsigned char> temp_image_depth(m_height, m_width);
    vpImageIo::read(temp_image_depth, filename_depth_image) ;
    vpImageConvert::convert(temp_image_depth, m_desired_depth_image);

//    vpImage<unsigned char> tmp_desired(m_height, m_width);
//    vpImageIo::read(tmp_desired, filename_read_image) ;
//    tmp_desired.halfSizeImage(m_desired_image);


//    vpImage<unsigned char> temp_mask(m_height, m_width);
//    m_vs_tools.read_mask(filename_image_mask, temp_mask);
//    temp_mask.halfSizeImage(m_mask_image);

    m_vs_tools.multiply_image_by_mask(m_desired_image, m_mask_image);
    m_logfile<<"initialization of visual servo"<<endl;



    //
    /// init current visual features
    //
    m_current_features.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_rho, REPTYPE, GRAPCALCTYPE);
    m_logfile<<"init done"<<endl;
    m_current_features.setCameraParameters(m_cam_param) ;
    m_current_features.set_DOF(true, false, false,
                               false, false, true); //Wx Wy Wz
    m_current_features.setInterpType(INTERPTYPE);

    //
    /// build desired features
    //
    m_logfile<<"initialization of visual servo done"<<endl;
    buildDesiredFeaturesWithDepth(m_desired_image, m_desired_depth_image, true, false, false,
                                                                            false, false, true, m_logfile);
//    m_vs_tools.build_desired_feature(m_desired_image, true, false, false,
//                                     false, false, true, m_desired_features, m_Lsd, m_Hsd, m_diagHsd, m_logfile);
    m_logfile<<"desired features built"<<endl;
}





void
VisualServoingWithKinect::buildDesiredFeaturesWithDepth(vpImage<unsigned char>& desired_image, vpImage<float>& depth,
                                                        const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                                                        const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ,
                                                        std::ostream& logfile)
{
    m_desired_features->setInterpType(INTERPTYPE);
    m_desired_features->init(desired_image.getHeight(), desired_image.getWidth(),
                             DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, m_rho, REPTYPE, GRAPCALCTYPE) ;
    m_logfile<<"init building desired features"<<endl;

    m_desired_features->set_DOF(DOF_VX, DOF_VY, DOF_VZ,
                                DOF_WX, DOF_WY, DOF_WZ);
    //
    // build visual feature from desired image
    //
    m_logfile<<"before building from depth image"<<endl;
    m_desired_features->buildFrom(desired_image, &depth);
    m_logfile<<"after building from depth image"<<endl;

    //
    // Compute interaction matrix at desired position
    //
    m_desired_features->interaction(this->m_Lsd) ;
    // Compute the Hessian H = L^TL
    m_Hsd = m_Lsd.AtA() ;
    // Compute the Hessian diagonal for the Levenberg-Marquardt
    unsigned int n = m_Lsd.getCols() ;
    m_diagHsd = vpMatrix(n,n);
    m_diagHsd.eye(n);
    for(unsigned int ii = 0 ; ii < n ; ii++)
    {
        m_diagHsd[ii][ii] = m_Hsd[ii][ii];
    }
}


void
VisualServoingWithKinect::imageCallbackuEye(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr_omni;
    try
    {
        m_cv_ptr_omni = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat flipped_image;
    cv::flip(m_cv_ptr_omni->image, flipped_image, 0);

    vpImageConvert::convert(flipped_image, m_current_image);


    //        this->m_current_image = visp_bridge::toVispImage(*image);
    m_vs_tools.multiply_image_by_mask(m_current_image, m_mask_image);
    vpDisplay::display(m_current_image);
    vpDisplay::flush(m_current_image);
    //
    // first mode rotate robot until finding the correct angle
    //
    vpColVector v, e;
    vpDisplay::display(m_desired_image);
    vpDisplay::flush(m_desired_image);

    vpImageTools::imageDifference(this->m_current_image, this->m_desired_image, this->m_difference_image) ;
    vpDisplay::display(this->m_difference_image);
    vpDisplay::flush(this->m_difference_image);

    m_current_features.buildFrom(m_current_image, &m_depth_image);

    vpColVector error;

    // ----------------------------------------------------------
    // Minimisation
    double lambdaGN = 10.0;
    // ----------------------------------------------------------
    int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations
    double normeError = 0;
    // compute current error

    m_current_features.error(*(m_desired_features), error);
    // ---------- Levenberg Marquardt method --------------
    if (this->m_iter == iterGN)
    {
        this->m_mu =  1e-6 ;
        m_lambda = lambdaGN;
    }

    vpMatrix H;

    H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();
    e = H*m_Lsd.t()*error;
    normeError = error.sumSquare();

    m_current_error = sqrt(normeError)/(m_current_image.getHeight()*m_current_image.getWidth());
    v = - m_lambda*e;

    m_velocity.linear.x = v[0];
    m_velocity.angular.z = v[1];
    m_ros_pub.publish(m_velocity);
    ros::spinOnce();

    //    cv::imshow("OpenCV viewer uEye RGB", flipped_image);
    //    //Enregister l'image ***************************************************
    //    char cc = cvWaitKey(33);
    //    if( cc == 32 ) //32 code ASCII "Espace" du clavier
    //    {
    //        vpImage<unsigned char> vp_flipped_image(m_height, m_width);
    //        vpImageConvert::convert(flipped_image, vp_flipped_image);
    //        stringstream string_des_image;
    //        string_des_image<<m_logs_path<<"desired_images/desired_image.png";
    //        vpImageIo::write(vp_flipped_image, string_des_image.str());
    //    }
    //    //*****************************************************************************
    //    cvWaitKey(3);

}



void
VisualServoingWithKinect::imageCallbackdepth(const sensor_msgs::ImageConstPtr& msg)
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

    m_logfile<<"minvalue depth"<<minVal<<"\tmax_value="<<maxVal<<endl;

    // visualize depth map
    cv::Mat depth_image;
    cv_ptr_depth->image.convertTo(depth_image, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));


    //        cv::imshow("kinect depth image", depth_image);
    //        cvWaitKey(3);

    /// Projection kinect-->omni
    //P (Majuscule): tout ce qui est en 3D
    //p (Miniscule): tout ce qui est en 2D

    //$^{c2}M_{c1}$ Matrice de transformation rigide: Kinect c1 -> Omni c2

    //Chargement du fichier xml de calibrage
    CModelStereo systeme(2);
    CModelStereoXml chargeSysteme("/home/yalj/MIXEDVISION_PO/MUI/bin64/calib.xml");
    chargeSysteme >> systeme;
    CPoint P;
    cv::Mat Imageprofomni(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, cv::DataType<float>::type);

    vpImage<float> depth_map(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, -1.0);
    Scalar c(1.0);
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

            //Projection perspective inverse dans le repère kinect (en mètre)
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
            int u_omni = vpMath::round(P.get_u());
            int v_omni = vpMath::round(P.get_v());

            if(u_omni>=0 && u_omni<this->m_cv_ptr_omni->image.cols && v_omni>=0 && v_omni<this->m_cv_ptr_omni->image.rows)
            {
                float depth_value = sqrt(pow(P.get_X(),2) + pow(P.get_Y(),2)+pow(P.get_Z(),2));
                Imageprofomni.at<float>(v_omni, u_omni) = depth_value;
                depth_map[v_omni][u_omni] = depth_value;
                m_logfile<<"v= "<<v_omni<<"u= "<<u_omni<<"\t"<<depth_value<<endl;
            }
            ptr_img_proj++;
        }
    }
    m_depth_image = depth_map;

    char cc = cvWaitKey(33);
    if( cc == 32 ) //32 code ASCII "Espace" du clavier
    {
        vpImage<unsigned char> res(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols);
        vpImageConvert::convert(m_depth_image, res);
        stringstream sstream;
        sstream<<m_logs_path<<"desired_images/desired_image_depth.png";
        vpImageIo::write(res, sstream.str().c_str());
    }


//    vpDisplayX dd(res, 10, 10, "res");
//    vpDisplay::display(res);
//    vpDisplay::flush(res);
//    vpDisplay::getClick(res);

    //    vpImageConvert::convert(Imageprofomni, m_depth_image);
    //   m_depth_image =
    //
    //
    //
    //    m_logfile<<"image \n"<<Imageprofomni<<endl;
    //    //m_logfile<<"size image_profondeur omni"<<Imageprofomni.rows<<"\t"<<Imageprofomni.cols<<endl;

    //    //****************************************************************************************

    //    //
    //    // conversion from float to 8UC
    //    //

    //    cv::Mat image8U_new(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, CV_8UC1);
    //    minMaxLoc(Imageprofomni, &minVal, &maxVal);
    //    //find minimum and maximum intensities
    //    Imageprofomni.convertTo(image8U_new, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    //    /// blending two images

    //    // Paramètres de superposition
    //    double alpha = 0.3; double beta;
    //    beta = ( 1.0 - alpha );

    //    /// Kinect+Omni

    //    Mat Image_Kinect_Omni;
    //    //Conversion de l'image kinect en 3 canaux
    //    cv::Mat image8U_rgb(this->m_cv_ptr_omni->image.rows, this->m_cv_ptr_omni->image.cols, DataType<Vec3b>::type);
    //    cvtColor(image8U_new, image8U_rgb, CV_GRAY2RGB);

    //    cv::Mat flipped_image;
    //    cv::flip(this->m_cv_ptr_omni->image, flipped_image, 0);

    //    //Superposition des deux images
    //    addWeighted(flipped_image, alpha, image8U_rgb, beta, 0.0, Image_Kinect_Omni);
    //    cv::imshow( "image kinect omni", Image_Kinect_Omni);

    //    //    Enregister l'image ***************************************************
    //    char cc = cvWaitKey(33);
    //    if( cc == 32 ) //32 code ASCII "Espace" du clavier
    //    {
    //        ROS_INFO("Space key is pressed");
    //        vpImage<vpRGBa> Imvisp_Kinect_Omni;
    //        vpImageConvert::convert (Image_Kinect_Omni, Imvisp_Kinect_Omni);
    //        vpImageIo::write(Imvisp_Kinect_Omni,"/home/yalj/data/Imvisp_Kinect_Omni.png");
    //    }
    //    //*****************************************************************************
    //    cvWaitKey(3);
}

void
VisualServoingWithKinect::imageCallbackrgb(const sensor_msgs::ImageConstPtr& msg)
{

    //cv_bridge::CvImagePtr cv_ptr_kinect_rgb;

    try
    {
        this->m_cv_ptr_kinect_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("OpenCV viewer Kinect RGB", this->m_cv_ptr_kinect_rgb->image);
    cvWaitKey(3);
}
