/*!
  \file CFeatureLuminanceOmni.cpp
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpImageConvert.h>


#include "photometricVS/CImageFilter.h"
#include "photometricVS/CFeatureLuminanceOmni.h"

//#include <PhOVS/CImageFilter.h>
//#include <PhOVS/CFeatureLuminanceOmni.h>

bool CFeatureLuminanceOmni::isNeighSet = false;
int *****CFeatureLuminanceOmni::Neigh = NULL;
bool CFeatureLuminanceOmni::isNeighSet_r = false;
float *****CFeatureLuminanceOmni::Neigh_r = NULL;
int CFeatureLuminanceOmni::di = 10;
int CFeatureLuminanceOmni::dj = 10 ;
int CFeatureLuminanceOmni::nbri = -10;
int CFeatureLuminanceOmni::nbrj = -10;
int CFeatureLuminanceOmni::pas = 1;
int CFeatureLuminanceOmni::imWidth = 0;
int CFeatureLuminanceOmni::imHeight = 0;
int CFeatureLuminanceOmni::derivativeMaskHalfSize = 3;
int CFeatureLuminanceOmni::nbNeigh = 6; //2*3
int CFeatureLuminanceOmni::nbDim = 0; //2*3
double CFeatureLuminanceOmni::deltaPSSampling = 0.4 * M_PI / 180.0;
double CFeatureLuminanceOmni::deltaCSSampling = 0.01;
CCameraOmniParameters CFeatureLuminanceOmni::cam(600,600,320,240,1) ;




/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
CFeatureLuminanceOmni::init()
{
    nbParameters = 1;
    dim_s = 0 ;
    /* di = dj = 10 ;
  nbri = nbrj = -di;
  pas = 1;*/
    flags = NULL;
    pixInfo = NULL;

    //imWidth = imHeight = 0;

    if (flags == NULL)
            flags = new bool[nbParameters];
    for (int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value rho (1 meters)
    rho = 1.;

    firstTimeIn =0 ;

    reptype = CARTIMAGEPLANE;
    gradcomptype = CLASSICAL;

    /*if(!isNeighSet)
    Neigh = NULL;

  derivativeMaskHalfSize = 3;
  nbNeigh = 6; //2*3

  deltaPSSampling = 0.4 * M_PI / 180.0;
  deltaCSSampling = 0.01;
*/
    nbDOF = 6;
    for(int i = 0 ; i < 6 ; i++) DOF[i] = true;
}

void
CFeatureLuminanceOmni::init(int _imHeight, int _imWidth, int _di, int _dj, int _nbri, int _nbrj, int _pas, vpImage<unsigned char> *Imask, double _rho, CRepresentationType _reptype, CGradientComputationType _gradcomptype, int _derivativeMaskSize)
{
    init() ;

    reptype = _reptype;
    gradcomptype = _gradcomptype;

    imWidth = _imWidth;
    imHeight = _imHeight;

    di = _di;
    dj = _dj;
    if((_nbri == di) && (imHeight > di))
        nbri = imHeight - di;
    else
        nbri = _nbri;
    if((_nbrj == dj) && (imWidth > dj))
        nbrj = imWidth - dj;
    else
        nbrj = _nbrj;

    pas =_pas;

    // number of feature = nb column x nb lines in the images
    dim_s = (nbri/pas)*(nbrj/pas);

    if(Imask != NULL)
    {
        unsigned char *pt_Imask = Imask->bitmap;
        long int i, j, nbi = imHeight, nbj = imWidth;

        for(i = 0 ; i < di ; i+=pas)
            for(j = 0 ; j < nbj ; j+=pas, pt_Imask+=pas);
        
        for(i = di ; i < (di+nbri) ; i+=pas)
        {
            for(j = 0 ; j < dj ; j+=pas, pt_Imask+=pas);

            for(j = dj ; j < (dj+nbrj) ; j+=pas, pt_Imask+=pas)
                if((*pt_Imask)==0)
                    dim_s--;

            for(j = dj+nbrj ; j < nbj ; j+=pas, pt_Imask+=pas);
        }
    }

    //std::cout << "dim_s : " << dim_s << std::endl;
    s.resize(dim_s) ;

    if (pixInfo != NULL)
        delete [] pixInfo;

    switch(reptype)
    {
    case CARTIMAGEPLANE:
        switch(gradcomptype)
        {
        case CLASSICAL: case DIRECT: case ADAPTED:
            nbDim = 2;
            break;
        default:
            nbDim = 0;
            break;
        }
        break;
    case POLARIMAGEPLANE:
        switch(gradcomptype)
        {
        case CLASSICAL:
        case DIRECT:
        case ADAPTED:
            nbDim = 2;
            break;
        default:
            nbDim = 0;
            break;
        }
        break;
    case CARTESIANSPHERICAL:
        switch(gradcomptype)
        {
        case CLASSICAL:
        case ADAPTED:
            nbDim = 2;
            break;
        case DIRECT:
            nbDim = 3;
            break;
        default:
            nbDim = 0;
            break;
        }
        break;
    case PURESPHERICAL:
        switch(gradcomptype)
        {
        case CLASSICAL:
        case DIRECT:
        case ADAPTED:
            nbDim = 2;
            break;
        default:
            nbDim = 0;
            break;
        }
        break;
    default:
        break;
    }

    switch(reptype)
    {
    case CARTIMAGEPLANE:
        pixInfo = new CLuminanceOmniCIP[dim_s];
        switch(gradcomptype)
        {
        case DIRECT:
        case ADAPTED:
            nbDim = 2;
            switch(interptype)
            {
            case IMAGEPLANE_BILINEAR:
                createNeighXYAdapt_r();
                break;
            case IMAGEPLANE_NEARESTNEIGH:
            default:
                createNeighXYAdapt();
                break;
            }
            break;
        case CLASSICAL:
            nbDim = 2;
        default:
            nbDim = 0;
            break;
        }
        break;
    case POLARIMAGEPLANE:
        pixInfo = new CLuminanceOmniPIP[dim_s] ;
        nbDim = 2;
        break;
    case CARTESIANSPHERICAL:
        pixInfo = new CLuminanceOmniCS[dim_s] ;
        switch(gradcomptype)
        {
        case ADAPTED:
            nbDim = 2;
            switch(interptype)
            {
            case IMAGEPLANE_BILINEAR:
                createNeighXYAdapt_r();
                break;
            case IMAGEPLANE_NEARESTNEIGH:
            default:
                createNeighXYAdapt();
                break;
            }
            break;
        case DIRECT:
            nbDim = 3;
            //A rendre dynamique au choix de l'utilisateur (delta=1pix dans l'iamge au centre, à l'extérieur, entre les deux -> un coef compris en 0 (centre) et 1 (bord)
        {
            double Xs0, Ys0, Zs0, Xs, Ys, Zs, x, y;
            cam.image2Sphere(0, 0, Xs0, Ys0, Zs0);
            vpPixelMeterConversion::convertPoint(cam, (double)cam.get_u0()+1, (double)cam.get_v0(), x, y);
            cam.image2Sphere(x, y, Xs, Ys, Zs);
            deltaCSSampling = sqrt(vpMath::sqr(Xs-Xs0) + vpMath::sqr(Ys-Ys0) + vpMath::sqr(Zs-Zs0));
            //std::cout << "deltaCSSampling : " << deltaCSSampling << std::endl;
        }
            switch(interptype)
            {
            case IMAGEPLANE_BILINEAR:
                createNeighXsYsZsSpher_r();
                break;
            case IMAGEPLANE_NEARESTNEIGH:
            default:
                createNeighXsYsZsSpher();
                break;
            }
            break;
        case CLASSICAL:
            nbDim = 2;
        default:
            nbDim = 0;
            break;
        }
        break;
    case PURESPHERICAL:
        pixInfo = new CLuminanceOmniPS[dim_s] ;
        switch(gradcomptype)
        {
        case ADAPTED:
            nbDim = 2;
            switch(interptype)
            {
            case IMAGEPLANE_BILINEAR:
                createNeighXYAdapt_r();
                break;
            case IMAGEPLANE_NEARESTNEIGH:
            default:
                createNeighXYAdapt();
                break;
            }
            break;
        case DIRECT:
            nbDim = 2;
            //A rendre dynamique au choix de l'utilisateur (delta=1pix dans l'iamge au centre, à l'extérieur, entre les deux -> un coef compris en 0 (centre) et 1 (bord)
        {
            vpColVector Xs0(3), Xs(3);
            double x, y;
            cam.image2Sphere(0, 0, Xs0[0], Xs0[1], Xs0[2]);
            vpPixelMeterConversion::convertPoint(cam, (double)cam.get_u0()+1, (double)cam.get_v0(), x, y);
            cam.image2Sphere(x, y, Xs[0], Xs[1], Xs[2]);
            double dotXs0Xs = vpColVector::dotProd(Xs0, Xs);
            deltaPSSampling = acos(dotXs0Xs);
            std::cout << "deltaPSSampling : " << deltaPSSampling << std::endl;
        }
            switch(interptype)
            {
            case IMAGEPLANE_BILINEAR:
                createNeighPhiThetaSpher_r();
                break;
            case IMAGEPLANE_NEARESTNEIGH:
            default:
                createNeighPhiThetaSpher();
                break;
            }
            break;
        case CLASSICAL:
            nbDim = 2;
        default:
            nbDim = 0;
            break;
        }
        break;
    default:
        break;
    }

    if(Imask != NULL)
    {
        unsigned char *pt_Imask = Imask->bitmap;
        long int i, j, nbi = imHeight, nbj = imWidth;

        for(i = 0 ; i < di ; i+=pas)
            for(j = 0 ; j < nbj ; j+=pas, pt_Imask+=pas);

        switch(reptype)
        {
        case CARTIMAGEPLANE:
        {
            CLuminanceOmniCIP *pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
            {
                for(j = 0 ; j < dj ; j+=pas, pt_Imask+=pas);

                for(j = dj ; j < (dj+nbrj) ; j+=pas, pt_Imask+=pas)
                    if((*pt_Imask)!=0)
                    {
                        pt_pixInfo->i = i;
                        pt_pixInfo->j = j;
                        pt_pixInfo++;
                    }

                for(j = (dj+nbrj) ; j < nbj ; j+=pas, pt_Imask+=pas);
            }
        }
            break;
        case POLARIMAGEPLANE:
        {
            CLuminanceOmniPIP *pt_pixInfo = (CLuminanceOmniPIP *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
            {
                for(j = 0 ; j < dj ; j+=pas, pt_Imask+=pas);

                for(j = dj ; j < (dj+nbrj) ; j+=pas, pt_Imask+=pas)
                    if((*pt_Imask)!=0)
                    {
                        pt_pixInfo->i = i;
                        pt_pixInfo->j = j;
                        pt_pixInfo++;
                    }

                for(j = (dj+nbrj) ; j < nbj ; j+=pas, pt_Imask+=pas);
            }
        }
            break;
        case CARTESIANSPHERICAL:
        {
            CLuminanceOmniCS *pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
            {
                for(j = 0 ; j < dj ; j+=pas, pt_Imask+=pas);

                for(j = dj ; j < (dj+nbrj) ; j+=pas, pt_Imask+=pas)
                    if((*pt_Imask)!=0)
                    {
                        pt_pixInfo->i = i;
                        pt_pixInfo->j = j;
                        pt_pixInfo++;
                    }

                for(j = (dj+nbrj) ; j < nbj ; j+=pas, pt_Imask+=pas);
            }
        }
            break;
        case PURESPHERICAL:
        {
            CLuminanceOmniPS *pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
            {
                for(j = 0 ; j < dj ; j+=pas, pt_Imask+=pas);

                for(j = dj ; j < (dj+nbrj) ; j+=pas, pt_Imask+=pas)
                    if((*pt_Imask)!=0)
                    {
                        pt_pixInfo->i = i;
                        pt_pixInfo->j = j;
                        pt_pixInfo++;
                    }

                for(j = (dj+nbrj) ; j < nbj ; j+=pas, pt_Imask+=pas);
            }
        }
            break;
        default:
            break;
        }
    }
    else
    {
        int i, j;

        switch(reptype)
        {
        case CARTIMAGEPLANE:
        {
            CLuminanceOmniCIP *pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
                for(j = dj ; j < (dj+nbrj) ; j+=pas)
                {
                    pt_pixInfo->i = i;
                    pt_pixInfo->j = j;
                    pt_pixInfo++;
                }
        }
            break;
        case POLARIMAGEPLANE:
        {
            CLuminanceOmniPIP *pt_pixInfo = (CLuminanceOmniPIP *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
                for(j = dj ; j < (dj+nbrj) ; j+=pas)
                {
                    pt_pixInfo->i = i;
                    pt_pixInfo->j = j;
                    pt_pixInfo++;
                }
        }
            break;
        case CARTESIANSPHERICAL:
        {
            CLuminanceOmniCS *pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
                for(j = dj ; j < (dj+nbrj) ; j+=pas)
                {
                    pt_pixInfo->i = i;
                    pt_pixInfo->j = j;
                    pt_pixInfo++;
                }
        }
            break;
        case PURESPHERICAL:
        {
            CLuminanceOmniPS *pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
            for(i = di ; i < (di+nbri) ; i+=pas)
                for(j = dj ; j < (dj+nbrj) ; j+=pas)
                {
                    pt_pixInfo->i = i;
                    pt_pixInfo->j = j;
                    pt_pixInfo++;
                }
        }
            break;
        default:
            break;
        }
    }

    rho = _rho ;

    //la taille est constante à 7 pour l'instant (7-1)/2 = 3
    derivativeMaskHalfSize = 3;//(_derivativeMaskSize-1)*0.5;
    nbNeigh = 2*derivativeMaskHalfSize;
}




//void CFeatureLuminanceOmni::setNumberOfParameters(unsigned int nb_params)
//{
//    this->nbParameters = nb_params;
//}

//void CFeatureLuminanceOmni::setRHO(double rho)
//{
//    this->rho =  rho;
//}


//void CFeatureLuminanceOmni::setFirstTimeIn( int firstimein)
//{
//    this->firstTimeIn = firstimein;
//}


//void CFeatureLuminanceOmni::createFlags()
//{
//        flags = new bool[nbParameters];
//}


//void CFeatureLuminanceOmni::setReptype(CRepresentationType reptype)
//{
//    this->reptype = reptype;
//}


//void CFeatureLuminanceOmni::setGradientComputationType(CGradientComputationType gradient_type)
//{
//   this->gradcomptype  = gradient_type;
//}

/*!
  To implement = operator for std::vector
*/

//CFeatureLuminanceOmni& CFeatureLuminanceOmni::operator = (const CFeatureLuminanceOmni &feature)
//{
//    return *this;
//}

/*!
  To implement for std library Copy constructor that build a visual feature
*/
//CFeatureLuminanceOmni::CFeatureLuminanceOmni(const CFeatureLuminanceOmni &feature)//: vpBasicFeature()
//{
////    this->set_rho(feature.get_rho);
////    this->setInterpType(feature);
//}





/*! 
  Default constructor that build a visual feature.
*/
CFeatureLuminanceOmni::CFeatureLuminanceOmni() : vpBasicFeature()
{
    interptype = IMAGEPLANE_NEARESTNEIGH;
    init() ;
}

/*! 
  Default destructor.
*/
CFeatureLuminanceOmni::~CFeatureLuminanceOmni() 
{
    if (pixInfo != NULL)
    {
        delete [] pixInfo ;
        pixInfo = NULL;
    }
    if (flags != NULL)
    {
        delete [] flags;
        flags = NULL;
    }
}

void
CFeatureLuminanceOmni::staticClean()
{
    if(Neigh != NULL)
        deleteNeigh();
    if(Neigh_r != NULL)
        deleteNeigh_r();
}

void
CFeatureLuminanceOmni::set_DOF(bool un, bool deux, bool trois, bool quatre, bool cinq, bool six)
{
    nbDOF = 0;
    DOF[0] = un; if(un) nbDOF++;
    DOF[1] = deux;  if(deux) nbDOF++;
    DOF[2] = trois;  if(trois) nbDOF++;
    DOF[3] = quatre;  if(quatre) nbDOF++;
    DOF[4] = cinq;  if(cinq) nbDOF++;
    DOF[5] = six;  if(six) nbDOF++;
}

/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
CFeatureLuminanceOmni::set_rho(const double rho)
{
    this->rho = rho ;
    flags[0] = true;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
CFeatureLuminanceOmni::get_rho() const
{
    return rho ;
}


void
CFeatureLuminanceOmni::setCameraParameters(CCameraOmniParameters &_cam) 
{
    cam = _cam ;
}

void
CFeatureLuminanceOmni::setInterpType(CInterpType _interptype)
{
    interptype=_interptype;
}


/*!

  Build a luminance feature directly from the image
*/

void
CFeatureLuminanceOmni::buildFrom(vpImage<unsigned char> &I)
{
    switch(reptype)
    {
    case CARTIMAGEPLANE:
        cartImagePlaneBuildFrom(I);
        break;
    case POLARIMAGEPLANE:
        polarImagePlaneBuildFrom(I);
        break;
    case CARTESIANSPHERICAL:
        cartesianSphericalBuildFrom(I);
        break;
    case PURESPHERICAL:
        pureSphericalBuildFrom(I);
        break;
    default:
        break;
    }
}

/*!

  Build a luminance feature directly from the image
*/

void
CFeatureLuminanceOmni::buildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd)
{
    if(sd == NULL)
        buildFrom(I);
    else
    {
        switch(reptype)
        {
        case CARTIMAGEPLANE:
            cartImagePlaneBuildFrom(I, sd);
            break;
        case POLARIMAGEPLANE:
            polarImagePlaneBuildFrom(I, sd);
            break;
        case CARTESIANSPHERICAL:
            cartesianSphericalBuildFrom(I, sd);
            break;
        case PURESPHERICAL:
            pureSphericalBuildFrom(I, sd);
            break;
        default:
            break;
        }
    }
}

/*!

  Build a luminance feature directly from the image
*/

void
CFeatureLuminanceOmni::buildFrom(vpImage<unsigned char> &I, vpImage<float> *Idepth)
{
    if(Idepth == NULL)
        buildFrom(I);
    else
    {
        switch(reptype)
        {
        case CARTIMAGEPLANE:
            cartImagePlaneBuildFrom(I, Idepth);
            break;
        case POLARIMAGEPLANE:
            polarImagePlaneBuildFrom(I, Idepth);
            break;
        case CARTESIANSPHERICAL:
            cartesianSphericalBuildFrom(I, Idepth);
            break;
        case PURESPHERICAL:
            pureSphericalBuildFrom(I, Idepth);
            break;
        default:
            break;
        }
    }
}

/*!

  Build a luminance feature directly from the image
*/

void
CFeatureLuminanceOmni::buildFrom(vpImage<unsigned char> &I, CFeatureLuminanceOmni *sd, vpImage<float> *Idepth)
{
    if(sd == NULL)
    {
        if(Idepth == NULL)
            buildFrom(I);
        else
            buildFrom(I, Idepth);
    }
    else
    {
        if(Idepth == NULL)
            buildFrom(I, sd);
        else
        {
            switch(reptype)
            {
            case CARTIMAGEPLANE:
                cartImagePlaneBuildFrom(I, sd, Idepth);
                break;
            case POLARIMAGEPLANE:
                polarImagePlaneBuildFrom(I, sd, Idepth);
                break;
            case CARTESIANSPHERICAL:
                cartesianSphericalBuildFrom(I, sd, Idepth);
                break;
            case PURESPHERICAL:
                pureSphericalBuildFrom(I, sd, Idepth);
                break;
            default:
                break;
            }
        }
    }
}

/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
CFeatureLuminanceOmni::interaction(vpMatrix &L)
{
    switch(reptype)
    {
    case CARTIMAGEPLANE:
        cartImagePlaneInteraction(L);
        break;
    case POLARIMAGEPLANE:
        polarImagePlaneInteraction(L);
        break;
    case CARTESIANSPHERICAL:
        cartesianSphericalInteraction(L);
        break;
    case PURESPHERICAL:
        pureSphericalInteraction(L);
        break;
    default:
        break;
    }
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix
CFeatureLuminanceOmni::interaction(unsigned int /* select */)
{
    static vpMatrix L  ;
    interaction(L);
    return L ;
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void
CFeatureLuminanceOmni::error(const vpBasicFeature &s_star,
                             vpColVector &e)
{
    e.resize(dim_s) ;
    double *pt_e = e.data, *pt_s = s.data;
    //int cpt = 0;
    switch(reptype)
    {
    case CARTIMAGEPLANE:
    {
        CLuminanceOmniCIP *pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
        for (int i =0 ; i < dim_s ; i++, pt_pixInfo++, pt_e++, pt_s++)
        {
            if(pt_pixInfo->notToProcessCurrently)
                *pt_e = 0.;
            else
            {
                *pt_e = *pt_s - s_star[i] ;
                //cpt++;
            }
        }
    }
        break;
    case POLARIMAGEPLANE:
    {
        CLuminanceOmniPIP *pt_pixInfo = (CLuminanceOmniPIP *)pixInfo;
        for (int i =0 ; i < dim_s ; i++, pt_pixInfo++, pt_e++, pt_s++)
        {
            if(pt_pixInfo->notToProcessCurrently)
                *pt_e = 0.;
            else
            {
                *pt_e = *pt_s - s_star[i] ;
                //cpt++;
            }
        }
    }
        break;
    case CARTESIANSPHERICAL:
    {
        CLuminanceOmniCS *pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
        for (int i =0 ; i < dim_s ; i++, pt_pixInfo++, pt_e++, pt_s++)
        {
            if(pt_pixInfo->notToProcessCurrently)
                *pt_e = 0.;
            else
            {
                *pt_e = *pt_s - s_star[i] ;
                //cpt++;
            }
        }
    }
        break;
    case PURESPHERICAL:
    {
        CLuminanceOmniPS *pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
        for (int i =0 ; i < dim_s ; i++, pt_pixInfo++, pt_e++, pt_s++)
        {
            if(pt_pixInfo->notToProcessCurrently)
                *pt_e = 0.;
            else
            {
                *pt_e = *pt_s - s_star[i] ;
                //cpt++;
            }
        }
    }
        break;
    default:
        break;
    }
    //std::cout << "CFeatureLuminanceOmni::error - cpt : " << cpt << std::endl;
}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector
CFeatureLuminanceOmni::error(const vpBasicFeature &s_star,
                             const int /* select */)
{
    static vpColVector e ;

    error(s_star, e) ;

    return e ;

}

/*!

  Not implemented.

 */


void
CFeatureLuminanceOmni::print(unsigned int /* select */) const
{
    static int firsttime =0 ;

    if (firsttime==0)
    {
        firsttime=1 ;
        vpERROR_TRACE("not implemented") ;
        // Do not throw and error since it is not subject
        // to produce a failure
    }
}



/*!

  Not implemented.

 */
void
CFeatureLuminanceOmni::display(const vpCameraParameters & cam ,
                               const vpImage<unsigned char> &  I ,
                               const vpColor& color ,  unsigned int  thickness ) const
{
    static int firsttime =0 ;

    if (firsttime==0)
    {
        firsttime=1 ;
        vpERROR_TRACE("not implemented") ;
        // Do not throw and error since it is not subject
        // to produce a failure
    }
}

/*!

  Not implemented.

 */
void
CFeatureLuminanceOmni::display(const vpCameraParameters &  cam,
                               const vpImage<vpRGBa> &  I,
                               const vpColor& color , unsigned int thickness ) const
{
    static int firsttime =0 ;

    if (firsttime==0)
    {
        firsttime=1 ;
        vpERROR_TRACE("not implemented") ;
        // Do not throw and error since it is not subject
        // to produce a failure
    }
}






/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
CFeatureLuminanceOmni*
CFeatureLuminanceOmni::duplicate() const
{
    CFeatureLuminanceOmni *feature = new CFeatureLuminanceOmni ;
    return feature ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
