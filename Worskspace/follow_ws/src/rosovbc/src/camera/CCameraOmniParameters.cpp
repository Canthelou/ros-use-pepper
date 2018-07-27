
/*!
  \file CCameraOmniParameters.cpp
  \brief Definition of the CCameraOmniParameters class member functions.
  Class CCameraOmniParameters define the camera intrinsic parameters

*/

#include <camera/CCameraOmniParameters.h>

//#include <PhOVS/CCameraOmniParameters.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>

CCameraOmniParameters::CCameraOmniParameters(const double px, const double py,
		     const double u0, const double v0, const double _xi)
{
initParamsCamOmni(px,py,u0,v0,_xi);
}

void CCameraOmniParameters::initParamsCamOmni(const double px, const double py,
                                      const double u0, const double v0, const double _xi)
{
  initPersProjWithoutDistortion(px,py,u0,v0);
  xi = _xi;
}

bool CCameraOmniParameters::image2SphereAngles(double x, double y, double & theta, double & phi)
{
	double rt = 1.0+(1.0-xi*xi)*(x*x + y*y);
	if(x && y && (rt >= 0) )
	{
		double Xs, Ys, Zs, fact = (xi + sqrt(rt)) / (x*x + y*y + 1.0);

		Xs = fact * x;
		Ys = fact * y;
		Zs = fact - xi;
	
		/*Xs = fact * x;
		Ys = -(fact - xi);
		Zs = fact * y;*/
	
		double hypotxy = sqrt(Xs*Xs+Ys*Ys);
		/*phi = atan2(Zs,hypotxy);
		theta = atan2(Ys,Xs);*/
	
		theta = atan2(Ys, Xs);
		//theta = atan(Ys / Xs);
		phi = acos(Zs);

		/*if(Xs >= 0)
			theta = asin(Ys/hypotxy);
		else
			theta = M_PI - asin(Ys/hypotxy);*/
	}
	else
	{
		theta = phi = -300;
		return false;
	}

	return true;
}

void CCameraOmniParameters::sphereAngles2Image(double theta, double phi, double & x, double & y)
{
	double Xs, Ys, Zs, fact;
	
	Xs = sin(phi)*cos(theta);
	Ys = sin(phi)*sin(theta);
	Zs = cos(phi);
	
	/*Zs = sin(phi);
	Xs = cos(phi) * cos(theta);
	Ys = cos(phi) * sin(theta);*/
	
	/*Ys = sin(phi);
	Xs = cos(phi) * cos(theta);
	Zs = -cos(phi) * sin(theta);*/
	
	fact = 1.0/(Zs + xi*sqrt(Xs*Xs + Ys*Ys + Zs*Zs));
	x = Xs * fact;
	y = Ys * fact;	
}

bool CCameraOmniParameters::image2Sphere(double x, double y, double & Xs, double & Ys, double & Zs)
{
	double rt = 1.0+(1.0-xi*xi)*(x*x + y*y);
	if(rt < 0.0)
		return false;
	
	double fact = (xi + sqrt(rt)) / (x*x + y*y + 1.0);
	
	Xs = fact * x;
	Ys = fact * y;
	Zs = fact - xi;	
	
	return true;
}

void CCameraOmniParameters::sphere2Image(double Xs, double Ys, double Zs, double & x, double & y)
{
	double fact = 1.0/(Zs + xi*sqrt(Xs*Xs + Ys*Ys + Zs*Zs)); //
	
	x = Xs * fact;
	y = Ys * fact;
}

bool CCameraOmniParameters::image2Sphere(double x, double y, vpColVector & Xs)
{
	double rt = 1.0+(1.0-xi*xi)*(x*x + y*y);
	if(rt < 0.0)
		return false;
	
	double fact = (xi + sqrt(rt)) / (x*x + y*y + 1.0);
	
	Xs[0] = fact * x;
	Xs[1] = fact * y;
	Xs[2] = fact - xi;	
	
	return true;
}

void CCameraOmniParameters::sphere2Image(vpColVector & Xs, double & x, double & y)
{
	double fact = 1.0/(Xs[2] + xi*sqrt(Xs[0]*Xs[0] + Xs[1]*Xs[1] + Xs[2]*Xs[2])); //
	
	x = Xs[0] * fact;
	y = Xs[1] * fact;
}
