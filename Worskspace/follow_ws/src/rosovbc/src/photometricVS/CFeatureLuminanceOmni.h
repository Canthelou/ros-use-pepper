/****************************************************************************
 *
 * July 2011
 *
 * Author:
 * Guillaume Caron
 * inspired from visp/vpFeatureLuminance (Eric Marchand)
 *
 *****************************************************************************/

#ifndef CFeatureLuminanceOmni_h
#define CFeatureLuminanceOmni_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>

#include "camera/CCameraOmniParameters.h"
//#include <PhOVS/CCameraOmniParameters.h>

/*!
  \file CFeatureLuminanceOmni.h
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/


/*!
  \class CLuminanceOmni
  \brief Class that defines the luminance and gradient of a point

  \sa CLuminanceOmni
*/


class CLuminanceOmni
{
public:
    int i, j;
    double I ; // pixel intensity
    double rho; // pixel depth
    bool toProcess, notToProcessCurrently; // boolean to precise if this pixel is to be processed or not (low gradient, M-Estimator, etc)

    CLuminanceOmni() { toProcess = true; notToProcessCurrently = false;}
};

//luminance in cartesian image plane
class CLuminanceOmniCIP : public CLuminanceOmni
{
public:
    double x, y;   // point coordinates (in meter)
    double Ix,Iy ; // pixel gradient
};

//luminance in polar image plane
class CLuminanceOmniPIP : public CLuminanceOmni
{
public:
    double zeta, alpha;   // point coordinates (meter, radian)
    double Izeta,Ialpha ; // pixel gradient
};

//luminance in pure spherical image
class CLuminanceOmniPS : public CLuminanceOmni
{
public:
    double phi, theta;   // point coordinates (in radian)
    double Iphi,Itheta ; // pixel gradient
};

//luminance in cartesian spherical image
class CLuminanceOmniCS : public CLuminanceOmni
{
public:
    double Xs, Ys, Zs;   // point coordinates (in meter)
    double IXs, IYs, IZs;// pixel gradient
};

/*!
  \class CFeatureLuminanceOmni
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/

class CFeatureLuminanceOmni : public vpBasicFeature
{
public:
    typedef enum{
        CARTIMAGEPLANE,
        POLARIMAGEPLANE,
        CARTESIANSPHERICAL,
        PURESPHERICAL
    } CRepresentationType;

    typedef enum{
        CLASSICAL, // (carré, régulier) suivi d'une élévation sur la sphère pour les CARTESIANSPHERICAL et PURESPHERICAL ou d'un changement de coordonnées pour POLARIMAGEPLANE
        ADAPTED, // (plan image adapté) suivi d'une élévation sur la sphère pour les CARTESIANSPHERICAL et PURESPHERICAL ou d'un changement de coordonnées pour POLARIMAGEPLANE
        DIRECT, // DIRECT = CLASSICAL pour la représentation CARTIMAGEPLANE, sinon, gradients directement calculés par rapport aux paramètres de la représentation
    } CGradientComputationType;

    typedef enum{
        IMAGEPLANE_NEARESTNEIGH,
        IMAGEPLANE_BILINEAR
    } CInterpType;

protected:
    //! FeaturePoint depth (required to compute the interaction matrix)
    //! default rho = 1m
    double rho ;

    static int imWidth, imHeight, di, nbri, dj, nbrj, pas;
    static int derivativeMaskHalfSize, nbNeigh;
    static int *****Neigh; //Neighborhood
    static float *****Neigh_r; //Neighborhood for intensity interpolation
    static int nbDim;
    static bool isNeighSet, isNeighSet_r;
    static double deltaPSSampling, deltaCSSampling;

    int nbDOF;
    bool DOF[6];

    //! Store the image (as a vector with intensity and gradient I, Ix, Iy)
    CLuminanceOmni *pixInfo ;
    int  firstTimeIn  ;

    //info representations
    CRepresentationType reptype;
    CGradientComputationType gradcomptype;
    CInterpType interptype;

public:
    void buildFrom(vpImage<unsigned char> &I) ;
    void buildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth) ;
    void buildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd) ;
    void buildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth) ;

private:
    //simple image feature
    void pureSphericalBuildFrom(vpImage<unsigned char> &I);
    void cartesianSphericalBuildFrom(vpImage<unsigned char> &I);
    void polarImagePlaneBuildFrom(vpImage<unsigned char> &I);
    void cartImagePlaneBuildFrom(vpImage<unsigned char> &I);

    //feature with depth
    void pureSphericalBuildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth);
    void cartesianSphericalBuildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth);
    void polarImagePlaneBuildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth);
    void cartImagePlaneBuildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth);

    //feature with desired knowledge
    void pureSphericalBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd);
    void cartesianSphericalBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd);
    void polarImagePlaneBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd);
    void cartImagePlaneBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd);

    //feature with desired knowledge and depth
    void pureSphericalBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth);
    void cartesianSphericalBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth);
    void polarImagePlaneBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth);
    void cartImagePlaneBuildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth);

    static void deleteNeigh();
    static void createNeighXYAdapt();
    static void createNeighPhiThetaSpher();
    static void createNeighXsYsZsSpher();

    static void deleteNeigh_r();
    static void createNeighXYAdapt_r();
    static void createNeighPhiThetaSpher_r();
    static void createNeighXsYsZsSpher_r();
public: 

    static void staticClean();

    void init() ;
    void init(int _imHeight, int _imWidth, int _di = 10, int _dj = 10, int _nbri = 10, int _nbrj = 10, int _pas = 1, vpImage<unsigned char> *Imask = NULL, double _rho = 1., CRepresentationType _reptype = CARTIMAGEPLANE, CGradientComputationType _gradcomptype = CLASSICAL, int _derivativeMaskSize = 7) ;


    //    void setNumberOfParameters(unsigned int nb_params);
    //    void setRHO(double new_rho);
    //    void setFirstTimeIn( int firstimein);
    //    void createFlags();
    //    void setReptype(CRepresentationType new_reptype);
    //    void setGradientComputationType(CGradientComputationType gradient_type);




    CFeatureLuminanceOmni() ;

    CFeatureLuminanceOmni(const CFeatureLuminanceOmni& feature) ;

    //! Destructor.
    virtual ~CFeatureLuminanceOmni() ;

public:
    static CCameraOmniParameters cam ;
    void setCameraParameters(CCameraOmniParameters &_cam) ;
    /*
    section Set/get Z
  */

    void setInterpType(CInterpType _interptype);
    void set_DOF(bool un = true, bool deux = true, bool trois = true, bool quatre = true, bool cinq = true, bool six = true);
    void set_rho(const double rho) ;
    double get_rho() const  ;
    CFeatureLuminanceOmni&   operator = (const CFeatureLuminanceOmni &feature);

    /*
    vpBasicFeature method instantiation
  */


    vpMatrix  interaction(unsigned int select = FEATURE_ALL);
    void      interaction(vpMatrix &L);
private:
    void pureSphericalInteraction(vpMatrix &L);
    void cartesianSphericalInteraction(vpMatrix &L);
    void polarImagePlaneInteraction(vpMatrix &L);
    void cartImagePlaneInteraction(vpMatrix &L);
public:
    vpColVector error(const vpBasicFeature &s_star,
                      const int select = FEATURE_ALL)  ;
    void error(const vpBasicFeature &s_star,
               vpColVector &e)  ;

    void print(unsigned int select = FEATURE_ALL ) const ;

    CFeatureLuminanceOmni *duplicate() const ;


    void display(const vpCameraParameters &cam,
                 const vpImage<unsigned char> &I,
                 const vpColor &color=vpColor::green,
                 unsigned int thickness=1) const;
    void display(const vpCameraParameters &cam,
                 const vpImage<vpRGBa> &I,
                 const vpColor &color=vpColor::green,
                 unsigned int thickness=1) const;


    //! Compute the error between a visual features and zero
    vpColVector error(const int select = FEATURE_ALL)  ;


} ;

#endif
