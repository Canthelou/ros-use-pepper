/*!
  \file CFeatureLuminanceOmni.cpp
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/

#include <visp/vpMatrix.h>

#include <photometricVS/CFeatureLuminanceOmni.h>
//#include <PhOVS/CFeatureLuminanceOmni.h>

void
CFeatureLuminanceOmni::pureSphericalInteraction(vpMatrix &L)
{
    double *pt_L;
    CLuminanceOmniPS *pt_pixInfo = (CLuminanceOmniPS *)pixInfo;
    bool *pt_DOF;

    double phi,theta,Iphi,Itheta;
    double cp, ct, sp, st;
    double rho, irho, cpsrho, unsrhosp, cpssp;

    int i;
    L.resize(dim_s, nbDOF, false);
    pt_L = L.data;

    for(int n = 0; n < dim_s; n++, pt_pixInfo++)
    {
        Iphi = pt_pixInfo->Iphi;
        Itheta = pt_pixInfo->Itheta;

        phi = pt_pixInfo->phi;
        theta = pt_pixInfo->theta;

        if(pt_pixInfo->notToProcessCurrently)
        {
            for(i =0 ; i < nbDOF ; i++, pt_L++) *pt_L = 0. ;
        }
        else
        {
            cp = cos(phi);
            sp = sin(phi);
            if( (sp != 0.0) )//&& (cp != 0.0))
            {
                ct = cos(theta);
                st = sin(theta);

                rho = pt_pixInfo->rho;
                irho = 1./rho;
                cpsrho = cp*irho;
                unsrhosp = irho/sp;
                cpssp = cp/sp;

                pt_DOF = DOF;
                if(*pt_DOF)     { *pt_L = -(Iphi * (-ct*cpsrho) + Itheta * (st*unsrhosp)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Iphi * (-st*cpsrho) + Itheta * (-ct*unsrhosp)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Iphi * (sp*irho)); pt_L++; } //    + Itheta * (0)
                if(*(++pt_DOF)) { *pt_L = -(Iphi * (st) + Itheta * (ct*cpssp)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Iphi * (-ct) + Itheta * (st*cpssp)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = Itheta; pt_L++; } //-(Iphi * (0)   + Itheta * (-1))
            }
            else
                for(i =0 ; i < nbDOF ; i++, pt_L++) *pt_L = 0. ;
        }
    }
}

void
CFeatureLuminanceOmni::cartesianSphericalInteraction(vpMatrix &L)
{
    double *pt_L;
    CLuminanceOmniCS *pt_pixInfo = (CLuminanceOmniCS *)pixInfo;
    bool *pt_DOF;

    double Xs, Ys, Zs, IXs, IYs, IZs;
    double rho, irho, XsYs, XsZs, YsZs;

    int i;
    L.resize(dim_s, nbDOF, false);
    pt_L = L.data;

    for(int n = 0; n < dim_s; n++, pt_pixInfo++)
    {
        IXs = pt_pixInfo->IXs;
        IYs = pt_pixInfo->IYs;
        IZs = pt_pixInfo->IZs;

        Xs = pt_pixInfo->Xs;
        Ys = pt_pixInfo->Ys;
        Zs = pt_pixInfo->Zs;

        if(pt_pixInfo->notToProcessCurrently)
        {
            for(i =0 ; i < nbDOF ; i++, pt_L++) *pt_L = 0. ;
        }
        else
        {
            rho = pt_pixInfo->rho;
            irho = 1./rho;
            XsYs = Xs*Ys;
            XsZs = Xs*Zs;
            YsZs = Ys*Zs;

            pt_DOF = DOF;
            if(*pt_DOF)     { *pt_L = -(IXs * (Xs*Xs-1.) + IYs * (XsYs) + IZs * (XsZs))*irho; pt_L++; }
            if(*(++pt_DOF)) { *pt_L = -(IXs * (XsYs) + IYs * (Ys*Ys-1.) + IZs * (YsZs))*irho; pt_L++; }
            if(*(++pt_DOF)) { *pt_L = -(IXs * (XsZs) + IYs * (YsZs) + IZs * (Zs*Zs-1.))*irho; pt_L++; } //    + Itheta * (0)
            if(*(++pt_DOF)) { *pt_L = -(IYs * (Zs) + IZs * (-Ys)); pt_L++; } //IXs * (0) +
            if(*(++pt_DOF)) { *pt_L = -(IXs * (-Zs) + IZs * (Xs)); pt_L++; } //+ IYs * (0)
            if(*(++pt_DOF)) { *pt_L = -(IXs * (Ys) + IYs * (-Xs)); pt_L++; } //+ IZs * (0)
        }
    }
}

void
CFeatureLuminanceOmni::polarImagePlaneInteraction(vpMatrix &L)
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

void
CFeatureLuminanceOmni::cartImagePlaneInteraction(vpMatrix &L)
{
    double *pt_L;
    CLuminanceOmniCIP *pt_pixInfo = (CLuminanceOmniCIP *)pixInfo;
    bool *pt_DOF;

    double xi = cam.get_xi();
    double x,y,Ix,Iy;
    double xi2 = xi*xi, unmxi2 = 1. - xi2, x2, y2;
    double rt, rho, irho, xisrho, xixysrho, rtpxi, irtpxi, irhortpxi, xsrho, ysrho, unmxirtpxi, xy;
    int i;//, cpt = 0;

    L.resize(dim_s, nbDOF, false);
    pt_L = L.data;

    for(int n = 0; n < dim_s; n++, pt_pixInfo++)
    {
        if(pt_pixInfo->notToProcessCurrently)
        {
            for(i =0 ; i < nbDOF ; i++, pt_L++) *pt_L = 0. ;
        }
        else
        {
            x = pt_pixInfo->x;
            y = pt_pixInfo->y;

            x2 = x*x;
            y2 = y*y;
            rt = 1.+unmxi2*(x2 + y2);
            if(rt >= 0)
            {
                Ix = pt_pixInfo->Ix;
                Iy = pt_pixInfo->Iy;
                rho = pt_pixInfo->rho;

                irho = 1./rho;
                rt = sqrt(rt);
                xisrho = xi*irho;
                xixysrho = x*y*xisrho;
                rtpxi = rt+xi;
                unmxirtpxi = 1.-xi*rtpxi;
                irtpxi = 1./rtpxi;
                irhortpxi = irtpxi*irho;
                xsrho = x*irho;
                ysrho = y*irho;

                xy = x*y;

                pt_DOF = DOF;
                if(*pt_DOF)     { *pt_L = -(Ix * (-(1. + x2*unmxirtpxi + y2)*irhortpxi) + Iy * xixysrho); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Ix * (xixysrho)                             + Iy * (-(1. + y2*unmxirtpxi + x2)*irhortpxi)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Ix * (rt*xsrho)                             + Iy * (rt*ysrho)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Ix * (xy)                                   + Iy * ((1. + y2)*rt - xi*x2)*irtpxi); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Ix * (-((1. + x2)*rt - xi*y2)*irtpxi)       + Iy * (-xy)); pt_L++; }
                if(*(++pt_DOF)) { *pt_L = -(Ix * (y)                                    + Iy * (-x)); pt_L++; }

                //cpt++;
            }
            else
                for(i =0 ; i < nbDOF ; i++, pt_L++) *pt_L = 0. ;
        }
    }
    //std::cout << "CFeatureLuminanceOmni::cartImagePlaneInteraction - cpt : " << cpt << std::endl;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
