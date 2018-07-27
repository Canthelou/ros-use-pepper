#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Twist.h"

//#include <visp/vpImage.h>
#include "visp_bridge/image.h"
#include <visp/vpDisplayX.h>

//#include "sensor_msgs/Image.h"
//#include "visp/vpImage.h"
//#include <visp_bridge/image.h>

//
// lib PhotoVS
//
#include <photometricVS/CImageTools.h>
#include <camera/CCameraOmniParameters.h>
#include <photometricVS/CFeatureLuminanceOmni.h>
#include <camera/CModel.h>
#include <camera/COmni.h>


//
// VISP
//
#include <stdlib.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpTime.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpList.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>
#include <visp/vpIoTools.h>
#include <visp/vpRobust.h>


////Pour images de synthèse 250x250
//#define DI 55
//#define DJ 130
//#define NBRI 420
//#define NBRJ 420
//#define PAS 1
//#define RHO 1.

//Pour images 656x656
#define DI 10
#define DJ 10
#define NBRI 633
#define NBRJ 633
#define PAS 1
#define RHO 1.


////Pour images 656x656
//#define DI 0
//#define DJ 0
//#define NBRI 655
//#define NBRJ 655
//#define PAS 1
//#define RHO 1.

//#define TOLZNCC 0.9975
#define TOLZNCC 0.949
#define TOLZNCCGN 0.9

/*#define REPTYPE CFeatureLuminanceOmni::CARTIMAGEPLANE
#define GRAPCALCTYPE CFeatureLuminanceOmni::CLASSICAL
#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_NEARESTNEIGH*/

#define REPTYPE CFeatureLuminanceOmni::CARTESIANSPHERICAL
#define GRAPCALCTYPE CFeatureLuminanceOmni::DIRECT
#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_NEARESTNEIGH

/*#define REPTYPE CFeatureLuminanceOmni::PURESPHERICAL
#define GRAPCALCTYPE CFeatureLuminanceOmni::DIRECT
#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_BILINEAR*/

#define CURRENT
#define INDICATORS
#define LISTINITPOSES
#define CSTRHO

// List of allowed command line options
#define GETOPTARGS  "cdi:h"

static const char WINDOW[] = "Image window";

using namespace std;

class PhotoVisualServo
{

public:

    PhotoVisualServo()
        : m_it(m_nh), m_logfile ("/home/yalj/catkin_ws/src/my_package/data/logfile.txt",ios::out)
    {
        string robotTopic;
        string cameraTopic;
        string pathDataFiles;

        m_nh.param("RobotTopic", robotTopic, string(""));
        m_nh.param("CameraTopic", cameraTopic, string(""));


        m_width = 656;
        m_height = 656;

        m_iter = 1;

        m_current_image.init(m_height,m_width);
        m_desired_image.init(m_height,m_width);
        m_difference_image.init(m_height, m_width);
        m_mask_image.init(m_height, m_width);

        m_display_current_image.init(m_current_image, 0, 0, "Current image");
        m_display_desired_image.init(m_desired_image, m_width+10, 0, "desired_image");
        m_display_error.init(m_difference_image, 2*m_width+10, 0, "Difference");

        initVisualServoing();
        //        m_image_sub = m_it.subscribe("/camera/image_color", 1, &PhotoVisualServo::imageCallback, this);
        //        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);

        m_nh.getParam("cameraTopic", cameraTopic);
        m_image_sub = m_it.subscribe(cameraTopic, 1, &PhotoVisualServo::imageCallback, this);

        m_nh.getParam("robotTopic",robotTopic);
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1000);

        ros::spinOnce();



    }

    ~PhotoVisualServo()
    {

    }


    void initVisualServoing()
    {

        std::string filename_read_image,filename_image_mask;

        filename_read_image = vpIoTools::path("/home/yalj/catkin_ws/src/my_package/data/desired_image.png");
        filename_image_mask  = vpIoTools::path("/home/yalj/catkin_ws/src/my_package/data/mask/mask.png");

        vpImageIo::read(this->m_desired_image, filename_read_image) ;
        vpImageIo::read(this->m_mask_image, filename_image_mask);

        //
        // camera model
        //
//        double px = 205.099;
//        double py = 205.887;
//        double u0 = 339.88;
//        double v0 = 265.56;
//        double xi = 1.1355;

        double px = 236.4569908;
        double py = 237.4384317;
        double u0 = 322.4879658;
        double v0 = 332.7940704;
        double xi = 0.959681985;

        double px_e = px, py_e = py, u0_e = u0, v0_e = v0, xi_e = xi;

        CCameraOmniParameters ccam(px_e, py_e, u0_e, v0_e, xi_e);

        double rho = RHO;

        //
        // desired visual feature built from the image
        //

        this->m_sId.setCameraParameters(ccam) ;

        this->m_sId.setInterpType(INTERPTYPE);

        this->m_sId.init(this->m_height, this->m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, rho, REPTYPE, GRAPCALCTYPE) ;
        m_logfile<<"after init"<<endl;
        this->m_sId.set_DOF(true, false, false, //Vx Vy Vz
                            false, false, true); //Wx Wy Wz


        //
        // build visual feature from desired image
        //
        this->m_sId.buildFrom(this->m_desired_image);
        m_logfile<<"after buildfrom"<<endl;


        //
        // Compute interaction matrix at desired position
        //
        this->m_sId.interaction(this->m_Lsd) ;

        m_logfile<<"after interaction"<<endl;

        // Compute the Hessian H = L^TL
        this->m_Hsd = this->m_Lsd.AtA() ;

        // Compute the Hessian diagonal for the Levenberg-Marquardt
        // optimization process

        unsigned int n = this->m_Lsd.getCols() ;
        m_logfile<<"n = "<<n<<endl;

        this->m_diagHsd = vpMatrix(n,n) ;
        this->m_diagHsd.eye(n);
        for(unsigned int i = 0 ; i < n ; i++)
            this->m_diagHsd[i][i] = this->m_Hsd[i][i];


        this->m_sI.init(this->m_height, this->m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask_image, rho, REPTYPE, GRAPCALCTYPE) ;
        this->m_sI.setCameraParameters(ccam) ;
        this->m_sI.set_DOF(true, false, false, //Vx Vy Vz
                           false, false, true); //Wx Wy Wz
        this->m_sI.setInterpType(INTERPTYPE);


        this->multiplyImageByMask(m_desired_image, m_mask_image);

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {

        m_logfile<<"start imageCallbak"<<endl;

        double t = vpTime::measureTimeMs();

        //
        // acquire image
        //
        this->m_current_image = visp_bridge::toVispImage(*image);

        this->multiplyImageByMask(m_current_image, m_mask_image);
        m_logfile<<"before t buildFrom 2"<<endl;
        this->m_sI.buildFrom(m_current_image) ;
        m_logfile<<"start buildFrom 2"<<endl;



        // ------------------------------------------------------
        // Control law
        double lambda ; //gain
        vpColVector e ;
        vpColVector v ; // camera velocity send to the robot


        // ----------------------------------------------------------
        // Minimisation

        double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
        double lambdaGN;


        mu       =  0.01;
        lambda   = 50;
        lambdaGN = 30;


        // ----------------------------------------------------------
        int iter   = 1;
        int iterGN = 90 ; // swicth to Gauss Newton after iterGN iterations




//        // ------------------------------------------------------
//        // Control law
//        double lambda ; //gain
//        vpColVector e ;
//        vpColVector v ; // camera velocity sent to the robot

//        // ----------------------------------------------------------
//        // Minimisation
//        double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
//        double lambdaGN;

//        mu       =  0.01;
//        lambda   = 30 ;
//        lambdaGN = 10;



//        // ----------------------------------------------------------
//        int iter   = 1;
//        int iterGN = 20 ; // swicth to Gauss Newton after iterGN iterations

//        // ------------------------------------------------------
//        // Control law
//        double lambda ; //gain
//        vpColVector e ;
//        vpColVector v ; // camera velocity sent to the robot

//        // ----------------------------------------------------------
//        // Minimisation
//        double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
//        double lambdaGN;

//        mu       =  0.01;
//        lambda   = 30 ;
//        lambdaGN = 20;

        // ----------------------------------------------------------
//        int iter   = 1;
//        int iterGN = 20 ; // swicth to Gauss Newton after iterGN iterations
        //        int iterMax = 50;

        double normeError = 0;

        //        std::string filename = "/home/yalj/catkin_ws/src/my_package/data/diff_image.png";
        //        filename_write_image = vpIoTools::path(filename);

        // compute current error
        this->m_sI.error(this->m_sId, this->m_error) ;

        // ---------- Levenberg Marquardt method --------------

        if (m_iter > iterGN)
        {
            mu = 0.0001 ;
            lambda = lambdaGN;
        }






        vpColVector gradient_du_cout;
        vpRobust robust(0);
        vpColVector w, weighted_error;

        int nerror;
        int nbPtsSuppressedByRobust, nbPtsOKByRobust, nb_good_data;

        int nbPoint = (int)(NBRI*NBRJ);

        vpColVector delta_s(nbPoint);
        //estimateur robuste
        if(iter == 1)
            weighted_error.resize(nbPoint);

        double num = 0, den = 0;
        nbPtsSuppressedByRobust = 0;
        nbPtsOKByRobust = 0;
        nb_good_data = 0;
        double wi, eri;

        int ind = 0;
        double r = 0.0;

        nerror = nbPoint;
        if(iter == 1)
        {
            w.resize(nerror);
            w = 1;

            //robust[icam].setThreshold(1e-4);
        }
        //cout << "toto " << iter << " " << delta_s.getRows() << " " << w.getRows() << endl;
        robust.MEstimator(vpRobust::TUKEY, delta_s, w);
        //cout << "toto" << endl;
        for(int i = 0 ; i < nerror ; i++)
        {
            wi = w[i];
            eri = delta_s[i];

            //if(wi < 0.7) wi = w[icam][i] = 0.0;

            num+=wi*vpMath::sqr(eri);
            den+=wi;

            if(wi != 0)
            {
                nb_good_data++;
                nbPtsOKByRobust++;
            }
            else
            {
                nbPtsSuppressedByRobust++;
            }
            weighted_error[ind++] = wi*eri;
        }

        unsigned int n = this->m_Lsd.getCols() ;
        vpMatrix Lt ;   // transposee de la matrice d'interaction
        vpMatrix Lp ;   // pseudo-inverse de la matrice d'interaction

        vpMatrix H;
        vpMatrix Id; // matrice identite
        vpMatrix L,id;


//        int j;
        ind = 0;
        for(unsigned int i = 0 ; i < delta_s.getRows() ; i++, ind++)
            for(unsigned int j = 0 ; j < n ; j++)
                L[ind][j] = w[i] * this->m_Lsd[ind][j];

        Lt = L.t();

        // Calcul du hessien a la position desiree
        m_Hsd = L.AtA() ;
        Id.eye(n);
        for(int i = 0 ; i < n ; i++) Id[i][i] = m_Hsd[i][i];

        gradient_du_cout = Lt * delta_s ;

        H = (mu * Id + m_Hsd).pseudoInverse();
        e = H * gradient_du_cout;
























        //Compute the levenberg Marquartd term
        //{

        H = ((mu * this->m_diagHsd) + this->m_Hsd).inverseByLU();

        //}
        //	compute the control law
        e = H * this->m_Lsd.t() *this->m_error;


        normeError = (this->m_error.sumSquare());
        double normalized_error = sqrt(normeError)/(this->m_height*this->m_width);
        m_logfile << "|e| "<< normalized_error<<std::endl ;


        v = - lambda*e;


        m_logfile << "lambda = " << lambda << "  mu = " << mu <<endl;
        m_logfile << "iter " << m_iter<<endl ;

        //        m_logfile << " |Tc| = " << sqrt(v.sumSquare()) << std::endl;


        t = vpTime::measureTimeMs() - t;
        m_logfile << "Time per frame "<<t<< std::endl;


        m_iter++;
        vpDisplay::display(m_current_image);
        vpDisplay::flush(m_current_image);
        vpImageIo::write(m_current_image, "/home/yalj/catkin_ws/src/my_package/data/myimage.png");

        vpImageTools::imageDifference(this->m_current_image,this->m_desired_image,this->m_difference_image) ;
        vpDisplay::display(this->m_difference_image);
        vpDisplay::flush(this->m_difference_image);
        vpImageIo::write(this->m_difference_image, "/home/yalj/catkin_ws/src/my_package/data/diff.png");

        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);


        if (normalized_error< 0.03)
        {
            m_velocity.linear.x = 0.0;
            m_velocity.angular.z = 0.0;
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
        }
        //
        // send the new velocity to the robot
        //

        //geometry_msgs::Twist newSpeed;
        m_logfile<<"v "<<v.t()<<std::endl;
        m_velocity.linear.x = v[0];
        m_velocity.angular.z = v[1];
        m_logfile<<endl;

        m_ros_pub.publish(m_velocity);
        ros::spinOnce();


    }


    void multiplyImageByMask(vpImage<unsigned char>& image, vpImage<unsigned char>& mask)
    {
        unsigned char *ptr_image =  image.bitmap;
        unsigned char *ptr_mask =  mask.bitmap;

        for (unsigned int i = 0; i<image.getNumberOfPixel(); i++, ptr_image++, ptr_mask++)
            *ptr_image = *ptr_image & *ptr_mask;

    }




private:
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    ros::Publisher m_ros_pub;

    vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_desired_image;
    vpImage<unsigned char> m_difference_image;
    vpImage<unsigned char> m_mask_image;

    CFeatureLuminanceOmni m_sId ;
    CFeatureLuminanceOmni m_sI ;

    vpMatrix m_Lsd;
    vpMatrix m_Hsd;
    vpMatrix m_diagHsd;

    vpColVector m_error ;

    int m_width;
    int m_height;

    int m_iter;
    geometry_msgs::Twist m_velocity;

    std::ofstream m_logfile;

    //    vpImage<unsigned char> my_image;
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;


};
