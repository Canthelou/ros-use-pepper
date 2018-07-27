#include "CModel.h"

CModel::CModel(double m_au,double m_av,double m_u0,double m_v0)
{
	init(m_au, m_av, m_u0, m_v0);
}

CModel::~CModel()
{
}

void CModel::init(double m_au,double m_av,double m_u0,double m_v0)
{
	name = "Perspective";
	au = m_au;
	av = m_av;
	u0 = m_u0;
	v0 = m_v0;
	inv_au = 1.0/au;
	inv_av = 1.0/av;
}

void CModel::init(const CModel &c)
{
	name = "Perspective";
	*this = c;
}

void CModel::meterPixelConversion(CPoint & P)
{

    P.set_u(P.get_x() * au + u0);
    P.set_v(P.get_y() * av + v0);


    //
    // YA
    //
    //P.setPixUV(P.get_x() * au + u0, P.get_y() * av + v0);
}

void CModel::pixelMeterConversion(CPoint & P)
{
    P.set_x((P.get_u() - u0) * inv_au);
    P.set_y((P.get_v() - v0) * inv_av);

    //
    // YA
    //

    //P.setPixUV((P.get_u() - u0) * inv_au, (P.get_v() - v0) * inv_av);
}

void CModel::setPrincipalPoint(double u0, double v0)
{
  this->u0    = u0 ;
  this->v0    = v0 ;
}

void CModel::setPixelRatio(double au, double av)
{
  this->au    = au ;
  this->av    = av ;
  
  this->inv_au = 1./au;
  this->inv_av = 1./av;
}

CModel& CModel::operator=(const CModel& cam)
{
  au = cam.au ;
  av = cam.av ;
  u0 = cam.u0 ;
  v0 = cam.v0 ;
  
  inv_au = cam.inv_au; 
  inv_av = cam.inv_av;

  return *this;
}

std::ostream& CModel::operator<<(std::ostream & os)
{
  os << "Camera parameters for generic model :" << std::endl ;
  os << "  au = " << getau() <<"\t av = "<< getav() << std::endl ;
  os << "  u0 = " << getu0() <<"\t v0 = "<< getv0() << std::endl ;
  
  return os;
}

vpMatrix CModel::getK() const
{
  vpMatrix K;

  K.resize(3,3) ;
  K = 0.0 ;
  K[0][0] = au ;
  K[1][1] = av ;
  K[0][2] = u0 ;
  K[1][2] = v0 ;
  K[2][2] = 1.0 ;
 
  return K; 
}
