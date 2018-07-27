#include "CPoint.h"

CPoint::CPoint()
{
	
}

CPoint::~CPoint()
{

}
void CPoint::getImageMetric(double &x, double &y, double &w)const
{
	x = get_x();
	y = get_y();
	w = get_w();
}

void CPoint::setImageMetric(const double x,const double y, const double w)
{
	set_x(x);
	set_y(y);
	set_w(w);
}

void CPoint::getPixUV(double &ui, double &vi)const
{
	ui = get_u();
	vi = get_v();
}

void CPoint::setPixUV(const double u, const double v)
{
	set_u(u);
	set_v(v);
}

void CPoint::setObjetImage(const double oX, const double oY, const double oZ, const double u, const double v)
{
	set_oX(oX);
	set_oY(oY);
	set_oZ(oZ);
	setPixUV(u,v);
}

CPoint& CPoint::operator=(const CPoint& pt)
{
	set_u(pt.get_u());
	set_v(pt.get_v());
	set_oX(pt.get_oX());
	set_oY(pt.get_oY());
	set_oZ(pt.get_oZ());
	set_X(pt.get_X());
	set_Y(pt.get_Y());
	set_Z(pt.get_Z());
	set_x(pt.get_x());
	set_y(pt.get_y());

	return *this;
}
