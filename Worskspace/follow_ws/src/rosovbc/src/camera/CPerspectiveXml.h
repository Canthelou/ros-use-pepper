#ifndef AFX_CPERSPXML
#define AFX_CPERSPXML

#include "CModelXml.h"
#include "CPerspective.h"

class CPerspectiveXml : public CModelXml
{
private:

public:	
	CPerspectiveXml(std::string fic):CModelXml(fic){};
	void operator<<(CPerspective&);
	void operator>>(CPerspective&);
};

#endif
