#ifndef AFX_CPerspective
#define AFX_CPerspective

#include "CModel.h"

class CPerspective : public CModel
{
	public:
//	CPerspective(CPerspective &c){ init(c); };
	CPerspective(double au=0,double av=0,double u0=0,double v0=0);
	~CPerspective();

	void project3DImage(CPoint &);

	//virtual friend std::ostream& operator << (std::ostream & os, const CPerspective &cam);	
	std::ostream& operator << (std::ostream & os);	
};

#endif
