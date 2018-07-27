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
#define DI 50
#endif

#ifndef DJ
#define DJ 50
#endif

#ifndef NBRI
#define NBRI 250 //596/2
#endif

#ifndef NBRJ
#define NBRJ 250 //596/2
#endif
#ifndef PAS
#define PAS 1
#endif


#define REPTYPE CFeatureLuminanceOmni::CARTESIANSPHERICAL
#define GRAPCALCTYPE CFeatureLuminanceOmni::DIRECT
#define INTERPTYPE CFeatureLuminanceOmni::IMAGEPLANE_NEARESTNEIGH

#define CURRENT
#define INDICATORS
#define LISTINITPOSES
#define CSTRHO

using namespace std;

class VisualServoTools
{

public:

    VisualServoTools();

    ~VisualServoTools();

    void clean();


    int getWidth(){return m_width;}
    int getHeight(){return m_height;}
    double getPx(){return m_px;}
    double getPy(){return m_py;}
    double getU0(){return m_u0;}
    double getV0(){return m_v0;}
    double getXi(){return m_xi;}
    double getRHO(){return m_rho;}

    int getDI(){return m_DI;}
    int getDJ(){return m_DJ;}
    int getNBRI(){return m_NBRI;}
    int getNBRJ(){return m_NBRJ;}

    CCameraOmniParameters& getCamParams(){return m_cam_param;}
    double getCurrentError(){return m_current_error;}
    double getPreviousError(){return m_prev_error;}

    vpMatrix& get_Lsd(){return m_Lsd;}
    vpMatrix& get_Hsd(){return m_Hsd;}
    vpMatrix& get_diagHsd(){return m_diagHsd;}


    void read_mask(string filename, vpImage<unsigned char>& mask);

    void set_mask(vpImage<unsigned char>& image);

    void multiply_image_by_mask(vpImage<unsigned char>& image,const vpImage<unsigned char>& mask);

    void bilateral_filter(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image);


    void canny(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image);





    void equalizeHisto(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image);

    void denoising(const vpImage<unsigned char>& input_image, vpImage<unsigned char>& output_image);

    void rotate(cv::Mat& src,const double center[2], const double angle, cv::Mat& dst, ostream& logfile = std::cout);

    void rotate_my_image(vpImage<unsigned char>& input_image, const double center[2], const double angle, vpImage<unsigned char>& output_image,
                         ostream& logfile = std::cout);


    // dst_image typically is the desired image src_image is the current image
    int find_best_angle( vpImage<unsigned char>& src_image_1,vpImage<unsigned char>& src_image_2,
                         const double rotation_center[2], const double step,
                         ostream& logfile = std::cout);


    // dst_image typically is the desired image src_image is the current image
    int find_best_angle_within_range( vpImage<unsigned char>& src_image_1,vpImage<unsigned char>& src_image_2,
                                      const double rotation_center[2], const double step, const double angle_min, const double angle_max,
                                      ostream& logfile = std::cout);



    //
    // routine that gathers all files with name 'filename' in directory 'path_to_directory' and stores their names in 'filenames'
    //
    void load_directory_and_sort_files(const char* path_to_directory, const char* filename_prefix, std::vector<string>& filenames,
                                       ostream& logfile = std::cout);

    //
    // read matrix from file and store it into matrix
    //
    void read_matrix_file(const string filename, vpMatrix& matrix);


    void init_visual_servo(CFeatureLuminanceOmni& si, const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                           const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile = std::cout);


    void build_desired_feature(vpImage<unsigned char>& desired_image,
                               const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                               const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile = std::cout
                               /*CFeatureLuminanceOmni* sId, vpMatrix& interaction, vpMatrix& Hsd, vpMatrix& diagHsd*/);

    void build_desired_feature(vpImage<unsigned char>& desired_image,
                               const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                               const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ,
                               CFeatureLuminanceOmni* sId, vpMatrix& interaction, vpMatrix& Hsd, vpMatrix& diagHsd,
                               ostream& logfile = std::cout);



    void build_desired_features_from_desired_images(std::vector<vpImage<unsigned char> >& desired_images,
                                                    const bool DOF_VX,const bool DOF_VY, const bool DOF_VZ,
                                                    const bool DOF_WX, const bool DOF_WY, const bool DOF_WZ, ostream& logfile = std::cout);




    void perform_a_VS_iteration_without_MEstimator(CFeatureLuminanceOmni& sI, /*CFeatureLuminanceOmni* sId,*/
                                                   vpImage<unsigned char>& current_image, vpColVector& v,
//                                                   const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                   ostream& logfile = std::cout);






    void perform_a_VS_iteration_without_MEstimator(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                   vpImage<unsigned char>& current_image, vpColVector& v,
                                                   const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                   ostream& logfile = std::cout);


    void perform_a_VS_iteration_without_MEstimator_without_changing_params(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                   vpImage<unsigned char>& current_image, vpColVector& v,
                                                   const vpMatrix& Lsd, const vpMatrix& Hsd, const vpMatrix& diagHsd,
                                                   ostream& logfile = std::cout);





    void perform_a_VS_iteration_with_MEstimator(CFeatureLuminanceOmni& sI,// CFeatureLuminanceOmni* sId,
                                                vpImage<unsigned char>& current_image, vpColVector& v,
//                                                const vpMatrix& Lsd, const vpMatrix& Hsd, const  vpMatrix& diagHsd,
                                                ofstream& logfile);


    void perform_a_VS_iteration_with_MEstimator(CFeatureLuminanceOmni& sI, CFeatureLuminanceOmni* sId,
                                                vpImage<unsigned char>& current_image, vpColVector& v,
                                                const vpMatrix& Lsd, const vpMatrix& Hsd, const  vpMatrix& diagHsd,
                                                ofstream& logfile);



    void write_matrices(vpMatrix &interaction, vpMatrix& Hsd, vpMatrix& diagHsd, int index, string data_path);


    void initialize();



    void init_params_with_MEstimator(std::ostream& ofs = std::cout);

    void init_params_without_MEstimator(std::ostream& ofs = std::cout);

    void increment_iteration();

    int get_iteration_number();

    void set_lambda(double lambda);
    void set_mu(double mu);


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
    template<typename T> std::vector<T> polyfit( const std::vector<T>& oX,
                                                 const std::vector<T>& oY, int nDegree );



    template<typename T>
    std::vector<T> poly_val_vec( const std::vector<T>& oCoeff,
                                 const std::vector<T>& oX);



    template<typename T>
    T& poly_val( const std::vector<T>& oCoeff,
                 const T& X);

protected :
    void init_iteration_number();


private:


    vpMatrix m_Lsd;
    vpMatrix m_Hsd;
    vpMatrix m_diagHsd;

    CFeatureLuminanceOmni* m_desired_features;
    std::vector<CFeatureLuminanceOmni*> m_vec_desired_features;
    std::vector<vpMatrix> m_vec_Lsd, m_vec_Hsd, m_vec_diagHsd;

    CCameraOmniParameters m_cam_param;
    vpImage<unsigned char> m_mask;
    double m_rho;
    int m_width;
    int m_height;
    double m_px;
    double m_py;
    double m_u0;
    double m_v0;
    double m_xi;
    double m_lambda;
    double m_mu;

    int m_DI;
    int m_DJ;
    int m_NBRI;
    int m_NBRJ;

    int m_iter;
    double m_current_error;
    double m_prev_error;

};


