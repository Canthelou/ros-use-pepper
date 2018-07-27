#ifndef vpSubPixel_h
#define vpSubPixel_h

#include "visp/vpConfig.h"

/*!
  \file vpSubPixel.h
  \brief Class that defines what is a sub-pixel.

*/


/*!
  \class vpSubPixel
  \brief Class that defines what is a sub-pixel.

  A sub-pixel is considered here as an image pixel with non-integer
  (u,v) coordinates.

  - Coordinate u stands for a position along the image horizontal axis.
  - Coordinate v stands for a position along the image vertical axis.

*/
class VISP_EXPORT vpSubPixel
{

 public:
  vpSubPixel();
  vpSubPixel(const double &u, const double &v);
  vpSubPixel(const vpSubPixel &p);
  virtual ~vpSubPixel() {}

  /*!

    Copy operator.

   */
  const vpSubPixel& operator=(const vpSubPixel &p) {
    this->u = p.u;
    this->v = p.v;
    return *this;
  }

  /*!
    Set the sub-pixel coordinate along the image horizontal axis.

    \sa set_v(const double &)
  */
  inline void set_u(const double &u) {this->u = u;}
  /*!
    Set the sub-pixel coordinate along the image vertical axis.

    \sa set_u(const double &)
  */
  inline void set_v(const double &v) {this->v = v;}
  
  /*!
    Get the sub-pixel coordinate along the image horizontal axis.

    \sa get_v()
  */
  inline double get_u() const {return u;}
  /*!
    Get the sub-pixel coordinate along the image vertival axis.

    \sa get_u()
  */
  inline double get_v() const {return v;}
  
  // Printing
  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,
						const vpSubPixel &p);

 private:
  double u; // Sub pixel coordinate along the horizontal axis
  double v; // Sub pixel coordinate along the vertical axis
  
} ;

#endif
