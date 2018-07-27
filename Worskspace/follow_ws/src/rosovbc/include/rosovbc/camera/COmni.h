#ifndef AFX_COmni
#define AFX_COmni

#include "CModel.h"

class COmni : public CModel
{
public://protected:
	double xi;

public:
	COmni(double au=0,double av=0,double u0=0,double v0=0,double xi=1);
	~COmni();

	void init(double au,double av,double u0,double v0,double xi);
    double getXi(){return xi;}
	void project3DImage(CPoint &);
	void project3DSphere(CPoint & P, double & Xs, double & Ys, double & Zs);
	virtual void projectSphereImage(CPoint &);
	virtual void projectImageSphere(CPoint &, double & Xs, double & Ys, double & Zs);
	
	COmni& operator=(const COmni& cam); 

};

#endif
