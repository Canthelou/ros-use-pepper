/*!
  \file CFeatureLuminanceOmni.cpp
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/

#include <visp/vpPixelMeterConversion.h>

#include <photometricVS/CImageFilter.h>
#include <photometricVS/CFeatureLuminanceOmni.h>

//#include <PhOVS/CImageFilter.h>
//#include <PhOVS/CFeatureLuminanceOmni.h>

void
CFeatureLuminanceOmni::pureSphericalBuildFrom(vpImage<unsigned char> &I)
{
    static int firsttime =0 ;

    double Ix, Iy, Iphi, Itheta;
    double px = cam.get_px(), py = cam.get_py(), xi = cam.get_xi();
    double f, dudT, dvdT, dudP, dvdP;
    double x=0,y=0, phi, theta;
    CLuminanceOmniPS *pt_pixInfo;
    double *pt_s;
    int i, j;
    if (firstTimeIn==0)
    {
        firstTimeIn=1 ;
        pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
        pt_s = s.data;

        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;
                vpPixelMeterConversion::convertPoint(cam,
                                                     j, i,
                                                     x, y);

                if(cam.image2SphereAngles(x, y, theta, phi))
                {
                    switch(gradcomptype)
                    {
                    case CLASSICAL:
                    case ADAPTED:
                        if(gradcomptype == CLASSICAL)
                        {
                            Ix =  CImageFilter::derivativeFilterX(I, i, j); //px *
                            Iy =  CImageFilter::derivativeFilterY(I, i, j); //py *
                        }
                        else
                        {
                            switch(interptype)
                            {
                            case IMAGEPLANE_BILINEAR:
                                Ix =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                                Iy =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                                break;
                            case IMAGEPLANE_NEARESTNEIGH:
                            default:
                                Ix =  CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                                Iy =  CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                                break;
                            }
                        }
                        f = sin(phi)/(cos(phi) + xi);
                        dudT = -px*sin(theta)*f;
                        dvdT = py*cos(theta)*f;
                        f = (1.+xi*cos(phi)) / vpMath::sqr(cos(phi)+xi);
                        dudP = px*cos(theta)*f;
                        dvdP = py*sin(theta)*f;

                        Itheta = Ix * dudT + Iy * dvdT;
                        Iphi = Ix * dudP + Iy * dvdP;
                        break;
                    case DIRECT:
                        switch(interptype)
                        {
                        case IMAGEPLANE_BILINEAR:
                            Iphi = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]) / deltaPSSampling; //VoisPhi
                            Itheta = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]) / deltaPSSampling; //VoisTheta
                            break;
                        case IMAGEPLANE_NEARESTNEIGH:
                        default:
                            Iphi = CImageFilter::derivativeFilter(I, i, j, Neigh[0]) / deltaPSSampling; //VoisPhi
                            Itheta = CImageFilter::derivativeFilter(I, i, j, Neigh[1]) / deltaPSSampling; //VoisTheta
                            break;
                        }
                        break;
                    default:
                        Iphi = Itheta = 0.0;
                        break;
                    }

                    pt_pixInfo->phi = phi;
                    pt_pixInfo->theta = theta;
                    pt_pixInfo->rho = rho;
                    *pt_s  =  I[i][j] ;
                    pt_pixInfo->I  = *pt_s;
                    pt_pixInfo->Iphi  = Iphi;
                    pt_pixInfo->Itheta  = Itheta;// / sin(phi);
                }
                else
                {
                    pt_pixInfo->toProcess = false;
                    pt_pixInfo->notToProcessCurrently = true;
                    *pt_s = 0.;
                }
            }
            else
            {
                *pt_s = 0.;
                pt_pixInfo->notToProcessCurrently = true;
            }
    }
    else
    {
        pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
        pt_s = s.data;
        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;
                switch(gradcomptype)
                {
                case CLASSICAL:
                case ADAPTED:
                    phi = pt_pixInfo->phi;
                    theta = pt_pixInfo->theta;
                    if(gradcomptype == CLASSICAL)
                    {
                        Ix =  CImageFilter::derivativeFilterX(I, i, j); //px *
                        Iy =  CImageFilter::derivativeFilterY(I, i, j); //py *
                    }
                    else
                    {
                        switch(interptype)
                        {
                        case IMAGEPLANE_BILINEAR:
                            Ix =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                            Iy =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                            break;
                        case IMAGEPLANE_NEARESTNEIGH:
                        default:
                            Ix =  CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                            Iy =  CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                            break;
                        }
                    }
                    f = sin(phi)/(cos(phi) + xi);
                    dudT = -px*sin(theta)*f;
                    dvdT = py*cos(theta)*f;
                    f = (1.+xi*cos(phi)) / vpMath::sqr(cos(phi)+xi);
                    dudP = px*cos(theta)*f;
                    dvdP = py*sin(theta)*f;

                    Itheta = Ix * dudT + Iy * dvdT;
                    Iphi = Ix * dudP + Iy * dvdP;
                    break;
                case DIRECT:
                    switch(interptype)
                    {
                    case IMAGEPLANE_BILINEAR:
                        Iphi = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]) / deltaPSSampling; //VoisPhi
                        Itheta = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]) / deltaPSSampling; //VoisTheta
                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:
                        Iphi = CImageFilter::derivativeFilter(I, i, j, Neigh[0]) / deltaPSSampling; //VoisPhi
                        Itheta = CImageFilter::derivativeFilter(I, i, j, Neigh[1]) / deltaPSSampling; //VoisTheta
                        break;
                    }
                    break;
                default:
                    Iphi = Itheta = 0.0;
                    break;
                }

                *pt_s  =  I[i][j] ;
                pt_pixInfo->I  = *pt_s;
                pt_pixInfo->Iphi  = Iphi;
                pt_pixInfo->Itheta  = Itheta;// / sin(phi);
            }
            else
                pt_pixInfo->notToProcessCurrently = true;
    }
}

void
CFeatureLuminanceOmni::cartesianSphericalBuildFrom(vpImage<unsigned char> &I)
{
    static int firsttime =0 ;

    double Ix, Iy, IXs, IYs, IZs;
    double px = cam.get_px(), py = cam.get_py(), u0 = cam.get_u0(), v0 = cam.get_v0(), xi = cam.get_xi();
    double f, dudXs, dvdYs, dudZs, dvdZs;
    double x=0,y=0, Xs, Ys, Zs;
    CLuminanceOmniCS *pt_pixInfo;
    double *pt_s;
    int i, j;//, cpt=0;
    if (firstTimeIn==0)
    {
        firstTimeIn=1 ;
        pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
        pt_s = s.data;

        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;
                vpPixelMeterConversion::convertPoint(cam,
                                                     j, i,
                                                     x, y);

                if(cam.image2Sphere(x, y, Xs, Ys, Zs))
                {
                    switch(gradcomptype)
                    {
                    case CLASSICAL:
                    case ADAPTED:
                        if(gradcomptype == CLASSICAL)
                        {
                            Ix =  CImageFilter::derivativeFilterX(I, i, j); //px *
                            Iy =  CImageFilter::derivativeFilterY(I, i, j); //py *
                        }
                        else
                        {
                            switch(interptype)
                            {
                            case IMAGEPLANE_BILINEAR:
                                Ix =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                                Iy =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                                break;
                            case IMAGEPLANE_NEARESTNEIGH:
                            default:
                                Ix =  CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                                Iy =  CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                                break;
                            }
                        }
                        f = 1. / (Zs + xi);
                        dudXs = px*f;
                        //dvdXs = 0.0;
                        //dudYs = 0.0;
                        dvdYs = py*f;
                        dudZs = -(j-u0)*f;
                        dvdZs = -(i-v0)*f;

                        IXs = Ix * dudXs; // + Iy * dvdXs;
                        IYs = Iy * dvdYs; //Ix * dudYs +
                        IZs = Ix * dudZs + Iy * dvdZs;
                        break;
                    case DIRECT:
                        switch(interptype)
                        {
                        case IMAGEPLANE_BILINEAR:
                            IXs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]) / deltaCSSampling; //VoisXs
                            IYs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]) / deltaCSSampling; //VoisYs
                            IZs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[2]) / deltaCSSampling; //VoisZs
                            break;
                        case IMAGEPLANE_NEARESTNEIGH:
                        default:
                            IXs = CImageFilter::derivativeFilter(I, i, j, Neigh[0]) / deltaCSSampling; //VoisXs
                            IYs = CImageFilter::derivativeFilter(I, i, j, Neigh[1]) / deltaCSSampling; //VoisYs
                            IZs = CImageFilter::derivativeFilter(I, i, j, Neigh[2]) / deltaCSSampling; //VoisZs
                            break;
                        }
                        break;
                    default:
                        IXs = IYs = IZs = 0.0;
                        break;
                    }

                    pt_pixInfo->Xs = Xs;
                    pt_pixInfo->Ys = Ys;
                    pt_pixInfo->Zs = Zs;
                    pt_pixInfo->rho = rho;
                    *pt_s  =  I[i][j] ;
                    pt_pixInfo->I  = *pt_s;
                    pt_pixInfo->IXs  = IXs;
                    pt_pixInfo->IYs  = IYs;
                    pt_pixInfo->IZs  = IZs;
                    //cpt++;
                }
                else
                {
                    pt_pixInfo->toProcess = false;
                    pt_pixInfo->notToProcessCurrently = true;
                    *pt_s = 0.;
                }
            }
            else
            {
                *pt_s = 0.;
                pt_pixInfo->notToProcessCurrently = true;
            }
    }
    else
    {
        pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
        pt_s = s.data;
        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;
                switch(gradcomptype)
                {
                case CLASSICAL:
                case ADAPTED:
                    if(gradcomptype == CLASSICAL)
                    {
                        Ix =  CImageFilter::derivativeFilterX(I, i, j); //px *
                        Iy =  CImageFilter::derivativeFilterY(I, i, j); //py *
                    }
                    else
                    {
                        switch(interptype)
                        {
                        case IMAGEPLANE_BILINEAR:
                            Ix =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                            Iy =  CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                            break;
                        case IMAGEPLANE_NEARESTNEIGH:
                        default:
                            Ix =  CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                            Iy =  CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                            break;
                        }
                    }
                    f = 1. / (Zs + xi);
                    dudXs = px*f;
                    //dvdXs = 0.0;
                    //dudYs = 0.0;
                    dvdYs = py*f;
                    dudZs = -(j-u0)*f;
                    dvdZs = -(i-v0)*f;

                    IXs = Ix * dudXs; // + Iy * dvdXs;
                    IYs = Iy * dvdYs; //Ix * dudYs +
                    IZs = Ix * dudZs + Iy * dvdZs;
                    break;
                case DIRECT:
                    switch(interptype)
                    {
                    case IMAGEPLANE_BILINEAR:
                        IXs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]) / deltaCSSampling; //VoisXs
                        IYs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]) / deltaCSSampling; //VoisYs
                        IZs = CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[2]) / deltaCSSampling; //VoisZs
                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:
                        IXs = CImageFilter::derivativeFilter(I, i, j, Neigh[0]) / deltaCSSampling; //VoisXs
                        IYs = CImageFilter::derivativeFilter(I, i, j, Neigh[1]) / deltaCSSampling; //VoisYs
                        IZs = CImageFilter::derivativeFilter(I, i, j, Neigh[2]) / deltaCSSampling; //VoisZs
                        break;
                    }
                    break;
                default:
                    IXs = IYs = IZs = 0.0;
                    break;
                }

                *pt_s  =  I[i][j] ;
                pt_pixInfo->I  = *pt_s;
                pt_pixInfo->IXs  = IXs;
                pt_pixInfo->IYs  = IYs;
                pt_pixInfo->IZs  = IZs;
            }
            else
                pt_pixInfo->notToProcessCurrently = true;
    }
}

void
CFeatureLuminanceOmni::polarImagePlaneBuildFrom(vpImage<unsigned char> &I)
{
    static int firsttime =0 ;

    if (firsttime==0)
    {
        firsttime=1 ;
        vpERROR_TRACE("not implemented") ;
        // Do not throw and error since it is not subject
        // to produce a failure
    }
    /*
      switch(gradcomptype)
      {
      case CLASSICAL:

        break;
      case ADAPTED:

        break;
      case DIRECT:

        break;
      default:
        break;
      }
*/
}

void
CFeatureLuminanceOmni::cartImagePlaneBuildFrom(vpImage<unsigned char> &I)
{
    double Ix,Iy ;
    double px = cam.get_px(), py = cam.get_py();
    double x=0,y=0,rhotmp;
    CLuminanceOmniCIP *pt_pixInfo;
    double *pt_s;
    int i, j;
    if (firstTimeIn==0)
    {
        firstTimeIn=1 ;
        pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
        pt_s = s.data;

        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
        {
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;
                vpPixelMeterConversion::convertPoint(cam,
                                                     j, i,
                                                     x, y);

                switch(gradcomptype)
                {
                case CLASSICAL:
                case DIRECT:
                    Ix =  px * CImageFilter::derivativeFilterX(I, i, j);
                    Iy =  py * CImageFilter::derivativeFilterY(I, i, j);
                    break;
                case ADAPTED:
                    switch(interptype)
                    {
                    case IMAGEPLANE_BILINEAR:
                        Ix =  px * CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                        Iy =  py * CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:
                        Ix =  px * CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                        Iy =  py * CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                        break;
                    }
                    break;
                default:
                    Ix = Iy = 0.0;
                    break;
                }

                pt_pixInfo->x = x;
                pt_pixInfo->y = y;
                pt_pixInfo->rho = rho;
                *pt_s  =  I[i][j] ;
                pt_pixInfo->I  = *pt_s;
                pt_pixInfo->Ix  = Ix;
                pt_pixInfo->Iy  = Iy;
            }
            else
                pt_pixInfo->notToProcessCurrently = true;
        }
    }
    else
    {
        pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
        pt_s = s.data;
        for(int n = 0 ; n < dim_s ; n++, pt_pixInfo++, pt_s++)
        {
            if(pt_pixInfo->toProcess)
            {
                pt_pixInfo->notToProcessCurrently = false;

                i = pt_pixInfo->i;
                j = pt_pixInfo->j;

                switch(gradcomptype)
                {
                case CLASSICAL:
                case DIRECT:
                    Ix =  px * CImageFilter::derivativeFilterX(I, i, j);
                    Iy =  py * CImageFilter::derivativeFilterY(I, i, j);
                    break;
                case ADAPTED:
                    switch(interptype)
                    {
                    case IMAGEPLANE_BILINEAR:
                        Ix =  px * CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[0]); //VoisX //px *
                        Iy =  py * CImageFilter::derivativeFilter_InterpBilinear(I, i, j, Neigh_r[1]); //VoisY //py *
                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:
                        Ix =  px * CImageFilter::derivativeFilter(I, i, j, Neigh[0]); //VoisX //px *
                        Iy =  py * CImageFilter::derivativeFilter(I, i, j, Neigh[1]); //VoisY //py *
                        break;
                    }
                    break;
                default:
                    Ix = Iy = 0.0;
                    break;
                }

                *pt_s  =  I[i][j] ;
                pt_pixInfo->I  =  *pt_s;
                pt_pixInfo->Ix  = Ix;
                pt_pixInfo->Iy  = Iy;
            }
            else
                pt_pixInfo->notToProcessCurrently = true;
        }
    }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
