#ifndef AFX_CPoint
#define AFX_CPoint

#include <iostream>
//#include "visp/vpSubPixel.h"
#include <camera/vpSubPixel.h>

#include "visp/vpPoint.h"

class CPoint:public vpPoint, public vpSubPixel
{
public:
	CPoint();
	~CPoint();
	void getImageMetric(double &x, double &y, double &w)const;
	void setImageMetric(const double x,const double y, const double w);
	void getPixUV(double &u, double &v)const;
	void setPixUV(const double u, const double v);
	void setObjetImage(const double oX, const double oY, const double oZ, const double u, const double v);

	CPoint& operator=(const CPoint&);
};

#endif
