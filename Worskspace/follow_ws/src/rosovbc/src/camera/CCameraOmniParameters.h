/*!
  \file CCameraOmniParameters.h
  \brief Declaration of the CCameraOmniParameters class.
  Class CCameraOmniParameters define the camera intrinsic parameters

*/

#ifndef CCAMERAOMNI_H
#define CCAMERAOMNI_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpColVector.h>

class CCameraOmniParameters : public vpCameraParameters
{
  //generic functions
public:
  CCameraOmniParameters() {}
  CCameraOmniParameters(const double px, const double py,
		     const double u0, const double v0, const double _xi) ;
  /*vpCameraOmniParameters(const double px, const double py,
                     const double u0, const double v0,
                     const double kud, const double kdu) ;
*/
  CCameraOmniParameters& operator =(const CCameraOmniParameters &c)
	{
		vpCameraParameters::operator=(c);
		this->xi = c.xi;
		
		return *this;
	}

  virtual ~CCameraOmniParameters() { }
/*
  void init() ;
  void init(const vpCameraOmniParameters &c) ;
  */
  void initParamsCamOmni(const double px, const double py,
                                      const double u0, const double v0, const double _xi) ;

	bool image2Sphere(double x, double y, vpColVector & Xs);
	void sphere2Image(vpColVector & Xs, double & x, double & y);
	bool image2Sphere(double x, double y, double & Xs, double & Ys, double & Zs);
	void sphere2Image(double Xs, double Ys, double Zs, double & x, double & y);
	
	bool image2SphereAngles(double x, double y, double & theta, double & phi);
	void sphereAngles2Image(double theta, double phi, double & x, double & y);
	
	/*
  void initPersProjWithoutDistortion(const double px, const double py,
                                      const double u0, const double v0) ;
  void initPersProjWithDistortion(const double px, const double py,
     const double u0, const double v0, const double kud,const double kdu) ;

  void printParameters() ;

  
  inline double get_px() const { return px; }
  inline double get_py() const { return py; }
  inline double get_u0() const { return u0; }
  inline double get_v0() const { return v0; }
  inline double get_kud() const { return kud; }
  inline double get_kdu() const { return kdu; }
   */
public:
  inline double get_xi() const { return xi; }
/*
  inline vpCameraOmniParametersProjType get_projModel() const { return projModel; } 
  
  vpMatrix get_K() const;

 //   @name Deprecated functions
  void init(const double px, const double py,
	    const double u0, const double v0) ;
  void setPixelRatio(const double px,const double py) ;
  void setPrincipalPoint(const double u0, const double v0) ;
*/
  void setXi(const double _xi) {xi = _xi;}
/*
private:
  static const double DEFAULT_U0_PARAMETER;
  static const double DEFAULT_V0_PARAMETER;
  static const double DEFAULT_PX_PARAMETER;
  static const double DEFAULT_PY_PARAMETER;
  static const double DEFAULT_KUD_PARAMETER;
  static const double DEFAULT_KDU_PARAMETER;
  static const vpCameraOmniParametersProjType DEFAULT_PROJ_TYPE; 


  double px, py ; //!< pixel size
  double u0, v0 ; //!<  principal point
  double kud ; //!< radial distortion (from undistorted to distorted)
  double kdu ; //!< radial distortion (from distorted to undistorted)
  */
private:
  double xi; //!< parametre du modele unifie
/*
  double inv_px, inv_py; 
   
  vpCameraOmniParametersProjType projModel ; //!< used projection model
*/
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
