#include "CPerspective.h"

CPerspective::CPerspective(double au,double av,double u0,double v0)
{
	name = "Perspective";
	type = Persp;
}

CPerspective::~CPerspective()
{

}

void CPerspective::project3DImage(CPoint &pt)
{
	pt.set_x(pt.get_X()/pt.get_Z());
	pt.set_y(pt.get_Y()/pt.get_Z());
	pt.set_w(1.0);
}

std::ostream& CPerspective::operator<<(std::ostream & os)
{
	os << "Camera parameters for perspective projection without distortion:" << std::endl;
	CModel::operator<<(os);

	return os;
}

