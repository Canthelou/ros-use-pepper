#ifndef AFX_CMODELSTEREOXML
#define AFX_CMODELSTEREOXML

#include <vector>

#include "CXml.h"
#include "CPerspectiveXml.h"
#include "COmniXml.h"
#include "CModelXml.h"
//#include "CParaboloidXml.h"
#include "CPerspective.h"
#include "COmni.h"
#include "CParaboloid.h"
#include "CModelStereo.h"

#define LABEL_XML_SYSTEM                         "system"
#define LABEL_XML_NBCAMS                         "NbCams"
#define LABEL_XML_NOCAM                           "NoCam"
#define LABEL_XML_POSE                             "pose"
#define LABEL_XML_TX                                 "tX"
#define LABEL_XML_TY                                 "tY"
#define LABEL_XML_TZ                                 "tZ"
#define LABEL_XML_THETAUX                          "thuX"
#define LABEL_XML_THETAUY                          "thuY"
#define LABEL_XML_THETAUZ                          "thuZ"


//!!GERER LES DIFFERENTS MODELES 
class CModelStereoXml : public CXml
{
protected:
	//ModelStereoXML
	xmlNodePtr putSystem(CModelStereo &);
	int getSystem(xmlNodePtr,CModelStereo &);
	int readSystem(CModelStereo &);
    int readNbCams(int &);
    int readCameraTypes(std::vector<ModelType> &);
	void writeSystem(CModelStereo &);

	//MireXML
	/*xmlNodePtr putMire(CMire);
	int getMire(xmlNodePtr,CMire);

	//ModelStereoXML
	xmlNodrPtr putModelStereo(*/

public:	
	CModelStereoXml(std::string fic):CXml(fic){};
	void operator<<(CModelStereo &);
    void operator>>(std::vector<ModelType> &);
    void operator>>(int &);
	void operator>>(CModelStereo &);
};

#endif
