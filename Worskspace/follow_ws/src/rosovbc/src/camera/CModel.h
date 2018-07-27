#ifndef AFX_CModel
#define AFX_CModel

#include "cv.h"
#include <iostream>
#include <string>
#include "CPoint.h"

#include "commun.h"

class CModel
{
public:
	double au,av,u0,v0,inv_av,inv_au;
	ModelType type;
	std::string name;
	
public:
	CModel(double au=0,double av=0,double u0=0,double v0=0);
	~CModel();

	void init(double au=0,double av=0,double u0=0,double v0=0);
	void init(const CModel& c);
	
	void meterPixelConversion(CPoint &);
	void pixelMeterConversion(CPoint &);
	virtual void project3DImage(CPoint &)=0;
	
	inline ModelType getType(){return type;}
	inline std::string getName() const { return name; }
	inline double getau() const { return au; }
	inline double getav() const { return av; }
	inline double getu0() const { return u0; }
	inline double getv0() const { return v0; }

	vpMatrix getK() const;
	
    inline void setType(ModelType m){ type=m;}
    inline void setName(std::string n) { name=n; }
	void setPixelRatio(double au, double av);
	void setPrincipalPoint(double u0, double v0);

	virtual CModel& operator=(const CModel& cam); //!< surcharge
	virtual std::ostream& operator << (std::ostream & os);
};
#endif
