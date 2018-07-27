#include "VisualServoTools.h"
#include <iostream>
//
// ROS
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//
// directory management
//
#include <dirent.h>
//
// LibPhotoVS
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
#include <visp/vpImageConvert.h>
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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>


#include <opencv2/photo/photo.hpp>

//Pour images de synthèse 250x250
//#ifndef DI
//#define DI 55/2
//#endif

//#ifndef DJ
//#define DJ 130/2
//#endif

//#ifndef NBRI
//#define NBRI 420/2
//#endif

//#ifndef NBRJ
//#define NBRJ 420/2
//#endif

//
// conditions :  DJ + NBRJ < height - DJ and DI + NBRI < height - DI
//

//#ifndef DI
//#define DI 15
//#endif

//#ifndef DJ
//#define DJ 15
//#endif

//#ifndef NBRI
//#define NBRI 299 //596/2
//#endif

//#ifndef NBRJ
//#define NBRJ 299 //596/2
//#endif


#ifndef DI
#define DI 10
#endif

#ifndef DJ
#define DJ 10
#endif

#ifndef NBRI
#define NBRI 620 
//350 //596/2
#endif

#ifndef NBRJ
#define NBRJ 460 
//350 //596/2
#endif
#ifndef PAS
#define PAS 2
#endif


#define REPTYPE CFeatureLuminanceOmni::CARTESIANSPHERICAL
#define GRAPCALCTYPE CFeatureLuminanceOmni::DIRECT
#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_NEARESTNEIGH

#define CURRENT
#define INDICATORS
#define LISTINITPOSES
#define CSTRHO

using namespace std;





 VisualServoTools::VisualServoTools()
    {
        m_desired_features =  new CFeatureLuminanceOmni();

        m_DI = 10;
        m_DJ = 65;
        m_NBRI = 255;
        m_NBRJ = 255;

/*
        // image 772
        m_px = 236.4569908;
        m_py = 237.4384317;
//        m_u0 = 386.4879658/2;
//        m_v0 = 386.7940704/2;
        m_u0 = 386.4879658;
        m_v0 = 386.7940704;

        m_xi = 0.959681985;
        m_rho = 1.0;
        */
        m_px = 300;
        m_py = 300;
//        m_u0 = 386.4879658/2;
//        m_v0 = 386.7940704/2;
        m_u0 = 320;
        m_v0 = 240;

        m_xi = 0.0;
        m_rho = 1.0;

        m_width = 640;
        m_height = 480;

//        m_width = 776/2;
//        m_height = 776/2;


        //        m_width = 690/2;
        //        m_height = 690/2;

        m_cam_param = CCameraOmniParameters(m_px, m_py, m_u0, m_v0, m_xi);
        m_iter = -1;

        //        m_current_error = 9999;
        //        m_iter          = -1;
        //        m_mu            = 0.00001;
        //        m_lambda  = 105;
    }


    VisualServoTools::~VisualServoTools()
    {
       delete m_desired_features;
    }

    void VisualServoTools::clean()
    {
        m_desired_features->staticClean();
        //        delete m_desired_features;
    }


    void VisualServoTools::read_mask(string filename, vpImage<unsigned char>& mask)
    {
        m_mask.init(m_height, m_width);
        vpImageIo::read(m_mask, filename);
        mask = m_mask;
    }

    void VisualServoTools::set_mask(vpImage<unsigned char>& image)
    {
        m_mask = image;
    }

    void VisualServoTools::multiply_image_by_mask(vpImage<unsigned char>& image,const vpImage<unsigned char>& mask)
    {

        unsigned char *ptr_image =  image.bitmap;
        unsigned char *ptr_mask =  mask.bitmap;

        for (unsigned int i = 0; i<image.getNumberOfPixel(); i++, ptr_image++, ptr_mask++)
            *ptr_image = *ptr_image & *ptr_mask;
    }

    void VisualServoTools::bilateral_filter(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image)
    {
        cv::Mat mat_img;
        cv::Mat mat_img_dst;
        vpImageConvert::convert(input_image,mat_img) ;

        //        int i = 44;
        int i = 6;
        bilateralFilter (mat_img, mat_img_dst, i, i*2, i/2 );

        //        GaussianBlur (mat_img, mat_img_dst, cv::Size( 3, 3 ),  0, 0);
        vpImageConvert::convert(mat_img_dst, output_image);
    }


    void VisualServoTools::canny(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image)
    {
        cv::Mat mat_img;
        cv::Mat mat_img_dst;
        vpImageConvert::convert(input_image,mat_img) ;
        // from ipl to cv::mat


        cv::Mat dst,detected_edges;
        //        int i = 44;
        int i = 10;
        blur (mat_img, detected_edges, cv::Size(3,3) );

        //        int edgeThresh = 1;
        int lowThreshold = 60;
        //        int const max_lowThreshold = 100;
        int ratio = 3;
        int kernel_size = 3;

        // Canny detector
        Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

        // Using Canny's output as a mask, we display our result
        dst = cv::Scalar::all(0);

        mat_img.copyTo( dst, detected_edges);

        vpImageConvert::convert(dst, output_image);
    }



    void VisualServoTools::equalizeHisto(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image)
    {
        cv::Mat mat_img;
        cv::Mat mat_img_dst;
        vpImageConvert::convert(input_image,mat_img) ;

        cv::equalizeHist(mat_img, mat_img_dst);
        vpImageConvert::convert(mat_img_dst, output_image);
    }

    void VisualServoTools::denoising(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image)
    {
        cv::Mat mat_img;
        cv::Mat mat_img_dst;
        vpImageConvert::convert(input_image,mat_img) ;

        cv::fastNlMeansDenoising(mat_img, mat_img_dst);
        vpImageConvert::convert(mat_img_dst, output_image);
    }

    void VisualServoTools::rotate(cv::Mat& src,const double center[2], const double angle, cv::Mat& dst, ostream& logfile)
    {
        //
        // check if the image is color
        //
        cv::Mat im_gray;


        //        cv::cvtColor(src,im_gray,CV_BGR2GRAY);

        logfile<<"###rotate image"<<endl;
        int len = std::max(src.cols, src.rows);
        //        cv::Point2f pt(328, 328);
        cv::Point2f pt(center[0], center[1]);
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
        cv::warpAffine(src, dst, r, cv::Size(len, len));
    }

    void VisualServoTools::rotate_my_image(vpImage<unsigned char>& input_image, const double center[2], const double angle,
                                           vpImage<unsigned char>& output_image, ostream& logfile)
    {
        cv::Mat cv_input_image, cv_dest_image;
        vpImageConvert::convert(input_image, cv_input_image);
        rotate(cv_input_image, center, angle, cv_dest_image);
        //        cv::imshow("fenetre",cv_dest_image);
        //        cv::waitKey(1);
        vpImageConvert::convert(cv_dest_image, output_image);
        //        vpDisplay::getClick(output_image);

        //        stringstream sstream;
        //        sstream<<m_logs_path<<"desired_images/desired_image_rot"<<angle<<".png";
        //        vpImageIo::write(input_image, sstream.str().c_str());
    }


    // dst_image typically is the desired image src_image is the current image
    int VisualServoTools::find_best_angle( vpImage<unsigned char>& src_image_1,vpImage<unsigned char>& src_image_2,
                         const double rotation_center[2], const double step, ostream& logfile)
    {
        cv::Mat mat_img_1;
        cv::Mat mat_img_2;

        vpImageConvert::convert(src_image_1,mat_img_1) ;
        vpImageConvert::convert(src_image_2,mat_img_2) ;

        //        vpImage <unsigned char> log_image(src_image_1.getHeight(), src_image_1.getWidth());

        cv::Mat mat_rotated;
        cv::Mat diff;
        double min = 9e20;
        int min_value;

        for (double i= 0.0; i<360.0; i=i+step)
        {
            logfile<<"visual compass\t"<<i<<endl;
            // rotate image
            rotate(mat_img_1, rotation_center, i, mat_rotated);
            //            cv::imshow("fenetre", mat_rotated);
            //            cv::waitKey(1);

            //
            // debug
            //

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

    // dst_image typically is the desired image src_image is the current image
    int VisualServoTools::find_best_angle_within_range( vpImage<unsigned char>& src_image_1,vpImage<unsigned char>& src_image_2,
                                      const double rotation_center[2], const double step, const double angle_min, const double angle_max,
                                      ostream& logfile)
    {
        cv::Mat mat_img;
        cv::Mat mat_dst;

        //convert to ipl
        vpImageConvert::convert(src_image_1,mat_img) ;
        vpImageConvert::convert(src_image_2,mat_dst) ;

        //        vpImage <unsigned char> log_image(src_image_1.getHeight(), src_image_1.getWidth());
        cv::Mat mat_rotated;
        cv::Mat diff;
        double min = 9e20;
        int min_value;

        for (double i= angle_min; i<angle_max; i=i+step)
        {
            // rotate image
            rotate(mat_img, rotation_center, i, mat_rotated);

            /*cv::imshow("fenetre",mat_rotated);
            cv::waitKey(1);
		*/
            //
            // debug
            //

            // compute difference
            cv::absdiff(mat_dst, mat_rotated, diff);
            cv::Scalar sc_sum_diff  = cv::sum(diff);
            double sum_dff  = sqrt(sc_sum_diff.val[0])/(src_image_1.getHeight()*src_image_1.getWidth());

            if (sum_dff < min)
            {
                min = sum_dff;
                min_value = i;
            }
        }

        if (min_value > 180.0)
        {
            min_value -= 360.0;
        }

        return min_value;
    }



    //
    // routine that gathers all files with name 'filename' in directory 'path_to_directory' and stores their names in 'filenames'
    //
    void VisualServoTools::load_directory_and_sort_files(const char* path_to_directory, const char* filename_prefix, std::vector<string>& filenames,
                                       ostream& logfile)
    {
        //std::vector<string> list_filenames;
        DIR *pDIR;
        struct dirent *entry;
        //enter to this directory

        logfile<<"path_to_directory "<<path_to_directory<<"filename_prefix"<<filename_prefix<<endl;
        if( pDIR = opendir(path_to_directory))
        {
            while(entry = readdir(pDIR))
            {
                string  filename = entry->d_name;
                // find files into current directory whose names contain the string "filename_prefix"
                std::size_t found = filename.find(filename_prefix);

                if (found!=std::string::npos)
                {
                    filenames.push_back(entry->d_name);
                    logfile<<"filename found "<<entry->d_name<<endl;
                }
            }
            closedir(pDIR);
        }
        std::sort(filenames.begin(), filenames.end());
    }

    //
    // read matrix from file and store it into matrix
    //
    void VisualServoTools::read_matrix_file(const string filename, vpMatrix& matrix)
    {
        ifstream ifs(filename.c_str(), std::ifstream::in);
        double a,b;
        int i=0,j=0;

        int nb_lines=0;
        string line;

        while (std::getline(ifs, line))
            nb_lines++;

        ifs.close();

        matrix.eye(nb_lines, 2);
        //        m_logfile<<"nb_lines"<<nb_lines<<endl;
        ifs.open(filename.c_str(), std::ifstream::in);

        for (int i= 0;i <nb_lines;i++ )
        {
            ifs>>a>>b;
            if( ifs.eof() ) break;
            matrix[i][0] = a;
            matrix[i][1] = b;
        }
        ifs.close();
    }

    void VisualServoTools::init_visual_servo(CFeatureLuminanceOmni& si, const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                           const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile)
    {        
        si.init(m_height, m_width, DI, DJ, NBRI, NBRJ, PAS, &m_mask, m_rho, REPTYPE, GRAPCALCTYPE);
        logfile<<"init done"<<endl;
        si.setCameraParameters(m_cam_param) ;
        si.set_DOF(DOF_VX,DOF_VY, DOF_VZ, //Vx Vy Vz
                   DOF_WX, DOF_WY, DOF_WZ); //Wx Wy Wz
        si.setInterpType(INTERPTYPE);

    }


    void VisualServoTools::build_desired_feature(vpImage<unsigned char>& desired_image,
                               const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                               const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile
            /*CFeatureLuminanceOmni* sId, vpMatrix& interaction, vpMatrix& Hsd, vpMatrix& diagHsd*/)
    {

        m_desired_features->setInterpType(INTERPTYPE);
        m_desired_features->init(desired_image.getHeight(), desired_image.getWidth(), DI, DJ, NBRI, NBRJ, PAS, &m_mask, m_rho,REPTYPE, GRAPCALCTYPE) ;
        m_desired_features->set_DOF(DOF_VX, DOF_VY, DOF_VZ,
                                    DOF_WX, DOF_WY, DOF_WZ);
        //
        // build visual feature from desired image
        //

        //        vpDisplay::getClick(desired_image);

        logfile<<"DI\t"<<DI<<"\tDJ\t"<<DJ<<"\tNBRIt"<<NBRI<<"\tNBRJ\t"<<NBRJ<<endl;
        m_desired_features->buildFrom(desired_image);
        //
        // Compute interaction matrix at desired position
        //
        m_desired_features->interaction(this->m_Lsd) ;
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

    }

    void VisualServoTools::build_desired_feature(vpImage<unsigned char>& desired_image,
                               const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                               const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ,
                               CFeatureLuminanceOmni* sId, vpMatrix& interaction, vpMatrix& Hsd, vpMatrix& diagHsd,
                               ostream& logfile)
    {

        sId->setInterpType(INTERPTYPE);

        sId->init(desired_image.getHeight(), desired_image.getWidth(),
                  DI, DJ, NBRI, NBRJ, PAS, &m_mask, m_rho, REPTYPE, GRAPCALCTYPE) ;

        sId->set_DOF(DOF_VX, DOF_VY, DOF_VZ,
                     DOF_WX, DOF_WY, DOF_WZ);



        //
        // build visual feature from desired image
        //

        //        vpDisplay::getClick(desired_image);

        //        logfile<<"DI\t"<<DI<<"\tDJ\t"<<DJ<<"\tNBRIt"<<NBRI<<"\tNBRJ\t"<<NBRJ<<endl;
        sId->buildFrom(desired_image);
        //
        // Compute interaction matrix at desired position
        //

        //logfile<<"desired features built"<<endl;

        sId->interaction(interaction) ;
        // Compute the Hessian H = L^TL
        Hsd = interaction.AtA() ;
        // Compute the Hessian diagonal for the Levenberg-Marquardt
        unsigned int n = interaction.getCols() ;
        diagHsd = vpMatrix(n,n) ;
        diagHsd.eye(n);
        for(unsigned int ii = 0 ; ii < n ; ii++)
        {
            diagHsd[ii][ii] = Hsd[ii][ii];
        }
//        logfile<<"matrices computed"<<endl;
    }


    void VisualServoTools::build_desired_features_from_desired_images(std::vector<vpImage<unsigned char> >& desired_images,
                                                    const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                                                    const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile)
    {

        unsigned int size_vec = desired_images.size();
        for(int i =0; i <size_vec; i++)
        {
            this->m_vec_desired_features[i] =  new CFeatureLuminanceOmni;
            this->m_vec_desired_features[i]->setInterpType(INTERPTYPE);
            this->m_vec_desired_features[i]->init(desired_images[i].getHeight(), desired_images[i].getWidth(),
                                                  DI, DJ, NBRI, NBRJ, PAS, &m_mask, m_rho, REPTYPE, GRAPCALCTYPE) ;
            this->m_vec_desired_features[i]->set_DOF(DOF_VX, DOF_VY, DOF_VZ,
                                                     DOF_WX, DOF_WY, DOF_WZ);
            //
            // build visual feature from desired image
            //

            //        vpDisplay::getClick(desired_image);

            logfile<<"DI\t"<<DI<<"\tDJ\t"<<DJ<<"\tNBRIt"<<NBRI<<"\tNBRJ\t"<<NBRJ<<endl;
            m_vec_desired_features[i]->buildFrom(desired_images[i]);
            //
            // Compute interaction matrix at desired position
            //
            m_vec_desired_features[i]->interaction(this->m_vec_Lsd[i]) ;
            // Compute the Hessian H = L^TL
            m_vec_Hsd[i] = this->m_vec_Lsd[i].AtA() ;
            // Compute the Hessian diagonal for the Levenberg-Marquardt
            unsigned int n = m_vec_Lsd[i].getCols() ;
            m_vec_diagHsd[i] = vpMatrix(n,n) ;
            m_vec_diagHsd[i].eye(n);
            for(unsigned int ii = 0 ; ii < n ; ii++)
            {
                m_vec_diagHsd[i][ii][ii] = m_vec_Hsd[i][ii][ii];
            }

        }

    }



    void VisualServoTools::perform_a_VS_iteration_without_MEstimator(CFeatureLuminanceOmni& sI, /*CFeatureLuminanceOmni* sId,*/
                                                   vpImage<unsigned char>& current_image, vpColVector& v,
                                                   //                                                   const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                   ostream& logfile)
    {
        vpColVector error;

        //
        // compute feature for each captured image
        //
        sI.buildFrom(current_image);

        //
        // generate velocity command
        //

        // Minimisation
        double lambdaGN = 5.0;
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations

        double normeError = 0.0;
        // compute current error
        sI.error(*(this->m_desired_features), error);


        // ---------- Levenberg Marquardt method --------------
        if (m_iter == iterGN)
        {
            m_mu =  0.0001 ;
            m_lambda = lambdaGN;
        }

        vpMatrix H;
        vpColVector e;


        if ((m_current_error < m_prev_error) && (m_iter > iterGN))
        {
            m_mu /=2;
        }

        logfile<<"# m_lambda"<<m_lambda<<"\tm_mu"<<m_mu<<"\titer"<<m_iter<<endl;
        H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();

        //	compute the control law
        e = H * (m_Lsd).t() *error;

        // Gauss Newton

        // e = m_Hsd.inverseByLU()* m_Lsd.t()*error;

        m_prev_error = m_current_error;

        normeError = (error.sumSquare());
        m_current_error = sqrt(normeError)/(current_image.getHeight()*current_image.getWidth());
        logfile << "|e| "<< m_current_error <<std::endl ;

        //m_logerror<<m_current_error<<endl;
        v = - m_lambda*e;
    }






    void VisualServoTools::perform_a_VS_iteration_without_MEstimator(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                   vpImage<unsigned char>& current_image, vpColVector& v,
                                                   const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                   ostream& logfile)
    {
        vpColVector error;

        //
        // compute feature for each captured image
        //
        logfile<<"before building features from current image"<<endl;
        sI.buildFrom(current_image);

        logfile<<"after building features from current image"<<endl;
        //
        // generate velocity command
        //

        // Minimisation
        double lambdaGN = 5.0;
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations

        double normeError = 0.0;
        // compute current error
        logfile<<"before error computtaion"<<endl;

        sI.error(*(sId), error);

        logfile<<"after error computtaion"<<endl;

        // ---------- Levenberg Marquardt method --------------
        if (m_iter == iterGN)
        {
            m_mu =  0.0001 ;
            m_lambda = lambdaGN;
        }

        vpMatrix H;
        vpColVector e;


        if ((m_current_error < m_prev_error) && (m_iter > iterGN))
        {
            m_mu /=2;
        }

        logfile<<"# m_lambda "<<m_lambda<<"\tm_mu "<<m_mu<<"\titer "<<m_iter<<endl;
        H = ((m_mu * (diagHsd)) +(Hsd)).inverseByLU();

        //	compute the control law
        e = H * (Lsd).t() *error;

        // Gauss Newton

        // e = m_Hsd.inverseByLU()* m_Lsd.t()*error;

        m_prev_error = m_current_error;

        normeError = (error.sumSquare());
        m_current_error = sqrt(normeError)/(current_image.getHeight()*current_image.getWidth());
        logfile << "|e| "<< m_current_error <<std::endl ;

        //m_logerror<<m_current_error<<endl;
        v = - m_lambda*e;
    }


    void VisualServoTools::perform_a_VS_iteration_without_MEstimator_without_changing_params(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                                           vpImage<unsigned char>& current_image, vpColVector& v,
                                                                           const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                                           ostream& logfile)
    {
        vpColVector error;

        //
        // compute feature for each captured image
        //
        sI.buildFrom(current_image);

        //
        // generate velocity command
        //

        // Minimisation
        double lambdaGN = 5.0;
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations

        double normeError = 0.0;
        // compute current error
        sI.error(*(sId), error);


        // ---------- Levenberg Marquardt method --------------


        vpMatrix H;
        vpColVector e;

        logfile<<"### lambda\t"<<m_lambda<<"\tm_mu\t"<<m_mu<<"\t iter\t"<<m_iter<<endl;
        H = ((m_mu * (diagHsd)) +(Hsd)).inverseByLU();

        //	compute the control law
        e = H * (Lsd).t() *error;

        // Gauss Newton

        // e = m_Hsd.inverseByLU()* m_Lsd.t()*error;

        m_prev_error = m_current_error;

        normeError = (error.sumSquare());
        m_current_error = sqrt(normeError)/(current_image.getHeight()*current_image.getWidth());
        logfile << "|e| "<< m_current_error <<std::endl ;

        //m_logerror<<m_current_error<<endl;
        v = - m_lambda*e;
    }
    //    void perform_a_VS_iteration_with_MEstimator(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
    //                                                       vpImage<unsigned char>& current_image, int& iter, double& lambda, double& mu, vpColVector& v,
    //                                                       const vpMatrix& Lsd, const vpMatrix& Hsd, const  vpMatrix& diagHsd,
    //                                                       double& current_error, double& pred_error, ofstream& logfile)





    void VisualServoTools::perform_a_VS_iteration_with_MEstimator(CFeatureLuminanceOmni& sI,// CFeatureLuminanceOmni* sId,
                                                vpImage<unsigned char>& current_image, vpColVector& v,
                                                //                                                const vpMatrix& Lsd, const vpMatrix& Hsd, const  vpMatrix& diagHsd,
                                                ofstream& logfile)

    {
        logfile<<"####### BEWARE BUILDING features from current image"<<endl;

        sI.buildFrom(current_image);
        vpColVector error;
        vpRobust robust(0);
        robust.setThreshold(0.0);
        vpColVector w;
        vpColVector weighted_error;

        // ----------------------------------------------------------
        // Minimisation
        double lambdaGN = 50.0;
        // ----------------------------------------------------------
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations
        double normeError = 0;
        // compute current error
        sI.error(*(m_desired_features),error);
        w.resize(error.getRows());
        weighted_error.resize(error.getRows());
        w =1;
        robust.MEstimator(vpRobust::TUKEY, error, w);
        // ---------- Levenberg Marquardt method --------------

        //        if (this->m_iter == iterGN)
        //        {
        //            this->m_mu =  1e-6 ;
        //            m_lambda = lambdaGN;
        //        }

        vpMatrix H;
        vpColVector e;

        //        if ((m_current_error < m_prev_error) && (m_iter > iterGN))
        //        {
        //            m_mu /=2;
        //        }

        //        vpMatrix temp_Lsd,temp_Hsd, temp_diagHsd;

        //        sI.interaction(temp_Lsd);

        ////        logfile<<temp_Lsd<<endl<<endl<<endl;

        ////        logfile<<m_Lsd<<endl<<endl<<endl;

        //        temp_Hsd = temp_Lsd.AtA() ;
        //        // Compute the Hessian diagonal for the Levenberg-Marquardt
        //        unsigned int n = temp_Lsd.getCols() ;
        //        temp_diagHsd = vpMatrix(n,n) ;
        //        temp_diagHsd.eye(n);
        //        for(unsigned int ii = 0 ; ii < n ; ii++)
        //        {
        //            temp_diagHsd[ii][ii] = temp_Hsd[ii][ii];
        //        }

        H = ((m_mu * (m_diagHsd)) +(m_Hsd)).inverseByLU();

        vpMatrix interaction_matrix ((m_Lsd).getRows(),(m_Lsd).getCols());
        int nb_total_points = (m_Lsd).getRows();

        for (int i=0; i<nb_total_points; i++)
        {
            weighted_error[i] = w[i]*error[i];
            interaction_matrix[i][0] = m_Lsd[i][0]*w[i];
            interaction_matrix[i][1] = m_Lsd[i][1]*w[i];
        }

        //	compute the control law
        e = H * interaction_matrix.t() * weighted_error;
        m_prev_error = m_current_error;
        normeError = weighted_error.sumSquare();
        m_current_error = sqrt(normeError)/(current_image.getHeight()*current_image.getWidth());
        logfile << "### |e|\t"<< m_current_error<<"\tpred_error\t"<< m_prev_error<<std::endl ;
        logfile<<"lambda\t"<<m_lambda<<"\tmu\t"<<m_mu<<endl;
        v = - m_lambda*e;
    }



    void VisualServoTools::perform_a_VS_iteration_with_MEstimator(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                vpImage<unsigned char>& current_image, vpColVector& v,
                                                const vpMatrix& Lsd, const vpMatrix& Hsd, const  vpMatrix& diagHsd,
                                                ofstream& logfile)

    {

        logfile<<"### build from current image"<<endl;

        sI.buildFrom(current_image);

        vpColVector error;
        vpRobust robust(0);
        robust.setThreshold(0.0);
        vpColVector w;
        vpColVector weighted_error;

        // ----------------------------------------------------------
        // Minimisation
        double lambdaGN = 50.0;
        // ----------------------------------------------------------
        int iterGN = 40 ; // swicth to Gauss Newton after iterGN iterations
        double normeError = 0;
        // compute current error

        logfile<<"### before error"<<endl;

        sI.error(*(sId),error);
        logfile<<"### error computed"<<endl;

        w.resize(error.getRows());
        logfile<<"### resize"<<endl;

        weighted_error.resize(error.getRows());
        w =1;
        robust.MEstimator(vpRobust::TUKEY, error, w);
        // ---------- Levenberg Marquardt method --------------

        //        if (this->m_iter == iterGN)
        //        {
        //            this->m_mu =  1e-6 ;
        //            m_lambda = lambdaGN;
        //        }


        vpMatrix H;
        vpColVector e;





        H = ((m_mu * (diagHsd)) +(Hsd)).inverseByLU();

        vpMatrix interaction_matrix ((Lsd).getRows(),(Lsd).getCols());
        int nb_total_points = (Lsd).getRows();

        for (int i=0; i<nb_total_points; i++)
        {
            weighted_error[i] = w[i]*error[i];
            interaction_matrix[i][0] = Lsd[i][0]*w[i];
            interaction_matrix[i][1] = Lsd[i][1]*w[i];
        }

        //	compute the control law
        e = H * interaction_matrix.t() * weighted_error;
        m_prev_error = m_current_error;
        normeError = weighted_error.sumSquare();
        m_current_error = sqrt(normeError)/(current_image.getHeight()*current_image.getWidth());
        logfile << "### |e|\t"<< m_current_error<<"\tpred_error\t"<< m_prev_error<<std::endl ;
        logfile<<"lambda\t"<<m_lambda<<"\tmu\t"<<m_mu<<endl;
        v = - m_lambda*e;

    }

    void VisualServoTools::write_matrices(vpMatrix &interaction, vpMatrix& Hsd, vpMatrix& diagHsd, int index, string data_path)
    {

        //
        // write data
        //
        stringstream sstr_interaction;
        stringstream sstr_Hsd;
        stringstream sstr_diagHsd;

        //
        // write interaction matrices into file
        //

        sstr_interaction<<data_path<<"interaction/interaction_"<<std::setw(4) << std::setfill('0')<<index<<".txt";
        ofstream ofs_interaction(sstr_interaction.str().c_str(),std::ofstream::out);

        ofs_interaction<<interaction;
        ofs_interaction.close();
        //
        // write Hsd matrices into file
        //
        sstr_Hsd<<data_path<<"Hsd/Hsd_"<<std::setw(4) << std::setfill('0')<<index<<".txt";
        ofstream ofs_Hsd(sstr_Hsd.str().c_str(),std::ofstream::out);
        ofs_Hsd<<Hsd;
        ofs_Hsd.close();

        //
        // write diagHsd into file
        //
        sstr_diagHsd<<data_path<<"diagHsd/diagHsd_"<<std::setw(4) << std::setfill('0')<<index<<".txt";
        ofstream ofs_diagHsd(sstr_diagHsd.str().c_str(),std::ofstream::out);
        ofs_diagHsd<<diagHsd<<endl;
        ofs_diagHsd.close();
    }

    //    void init_values_with_MEstimator(double& current_error, int& iter, double& mu, double& lambda, std::ostream& ofs = std::cout)
    //    {
    //        current_error = 9999;
    //        iter          = 0;
    //        mu            = 0.00001;
    ////        lambda        = 40.0;
    ////        lambda        = 350;
    //        lambda  = 105;
    ////        lambda = 15;
    //        ofs<<"current_error"<<current_error<<"\titer"<<iter<<"\tmu="<<mu<<"\tlmabda="<<lambda<<endl;
    //    }

    void VisualServoTools::initialize()
    {
        m_current_error = 9999.99;
        init_iteration_number();
    }



    void VisualServoTools::init_params_with_MEstimator(std::ostream& ofs)
    {
        //        m_current_error = 9999;
        //        m_mu            = 0.00001;
        //        m_mu  = 0.1;
        //        m_mu  = 0.001;

        //        m_lambda = 55;
        //        m_lambda = 1050;

        //        m_lambda = 500;
        //        m_lambda  = 1200;
        //        iter          = 0;
        //        lambda        = 40.0;
        //        lambda        = 350;
        //        lambda = 15;
        //        ofs<<"Call to Init params with MEstim: current_error"<<m_current_error<<"\tmu= "<<m_mu<<"\tlmabda= "<<m_lambda<<endl;
    }

    void VisualServoTools::init_params_without_MEstimator(std::ostream& ofs)
    {
        m_current_error = 9999;
        m_mu        = 0.001;
        m_lambda  = 10;
        ofs<<"Call to Init params without  MEstim: current_error"<<m_current_error<<"\tmu="<<m_mu<<"\tlmabda= "<<m_lambda<<endl;
    }


    void VisualServoTools::increment_iteration()
    {
        m_iter++;
    }

    int VisualServoTools::get_iteration_number()
    {
        return m_iter;
    }

    void VisualServoTools::set_lambda(double lambda)
    {
        m_lambda  = lambda;
    }

    void VisualServoTools::set_mu(double mu)
    {
        m_mu = mu;
    }

    //    void init_values_without_MEstimator(double& current_error, int& iter, double& mu, double& lambda,std::ostream& ofs = std::cout)
    //    {
    //        current_error = 9999;
    //        iter      = 0;
    //        mu        = 0.001;
    //        lambda  = 20;
    //        ofs<<"current_error"<<current_error<<"\titer"<<iter<<"\tmu="<<mu<<"\tlmabda="<<lambda<<endl;
    //    }

    /*
      * fitting data (ox,oy) with a polynomial of degree n (requires boost)
      * from http://vilipetek.com/2013/10/07/polynomial-fitting-in-c-using-boost/
      ***************************************************************************/
    // the first element of the vector corresponds to the lowest exponent
    template<typename T>
    std::vector<T> VisualServoTools::polyfit( const std::vector<T>& oX,
                                                 const std::vector<T>& oY, int nDegree )
    {
        using namespace boost::numeric::ublas;

        if ( oX.size() != oY.size() )
            throw std::invalid_argument( "X and Y vector sizes do not match" );

        // more intuative this way
        nDegree++;

        size_t nCount =  oX.size();
        matrix<T> oXMatrix( nCount, nDegree );
        matrix<T> oYMatrix( nCount, 1 );

        // copy y matrix
        for ( size_t i = 0; i < nCount; i++ )
        {
            oYMatrix(i, 0) = oY[i];
        }

        // create the X matrix
        for ( size_t nRow = 0; nRow < nCount; nRow++ )
        {
            T nVal = 1.0f;
            for ( int nCol = 0; nCol < nDegree; nCol++ )
            {
                oXMatrix(nRow, nCol) = nVal;
                nVal *= oX[nRow];
            }
        }

        // transpose X matrix
        matrix<T> oXtMatrix( trans(oXMatrix) );
        // multiply transposed X matrix with X matrix
        matrix<T> oXtXMatrix( prec_prod(oXtMatrix, oXMatrix) );
        // multiply transposed X matrix with Y matrix
        matrix<T> oXtYMatrix( prec_prod(oXtMatrix, oYMatrix) );

        // lu decomposition
        permutation_matrix<int> pert(oXtXMatrix.size1());
        const std::size_t singular = lu_factorize(oXtXMatrix, pert);
        // must be singular
        BOOST_ASSERT( singular == 0 );

        // backsubstitution
        lu_substitute(oXtXMatrix, pert, oXtYMatrix);

        // copy the result to coeff
        return std::vector<T>( oXtYMatrix.data().begin(), oXtYMatrix.data().end() );
    }


    template<typename T>
    std::vector<T> VisualServoTools::poly_val_vec( const std::vector<T>& oCoeff,
                                 const std::vector<T>& oX )
    {
        size_t nCount =  oX.size();
        size_t nDegree = oCoeff.size();
        std::vector<T>    oY( nCount );

        for ( size_t i = 0; i < nCount; i++ )
        {
            T nY = 0;
            T nXT = 1;
            T nX = oX[i];
            for ( size_t j = 0; j < nDegree; j++ )
            {
                // multiply current x by a coefficient
                nY += oCoeff[j] * nXT;
                // power up the X
                nXT *= nX;
            }
            oY[i] = nY;
        }
        return oY;
    }


    template<typename T>
    T& VisualServoTools::poly_val( const std::vector<T>& oCoeff, const T& X )
    {
        size_t nDegree = oCoeff.size();

        T nY = 0;
        T nXT = 1;
        T nX = X;
        for ( size_t j = 0; j < nDegree; j++ )
        {
            // multiply current x by a coefficient
            nY += oCoeff[j] * nXT;
            // power up the X
            nXT *= nX;
        }
        return nY;

    }


    void VisualServoTools::init_iteration_number()
    {
        m_iter = -1;
    }




