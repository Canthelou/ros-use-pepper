/*!
  \file CFeatureLuminanceOmni.cpp
  \brief Class that defines the omnidirectional image luminance visual feature for several parameterizations

  for more details see
  G. Caron, E. Marchand, E. Mouaddib, IROS'2010
*/

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <photometricVS/CFeatureLuminanceOmni.h>

//#include <PhOVS/CFeatureLuminanceOmni.h>

//Integer neighborhoods

//Neigh[0] <=> VoisX
//Neigh[1] <=> VoisY

void
CFeatureLuminanceOmni::createNeighXYAdapt()
{
    if(isNeighSet)
        return;

    isNeighSet = true;

    //Voisinage 2D
    Neigh = new int****[2];

    int nba = 2, h, l, n, r, c, p, j, k;

    double u0 = cam.get_u0(), v0 = cam.get_v0();

    Neigh[0] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[0][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[0][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[0][h][l][n] = new int[nbNeigh];
            }
        }
    }

    Neigh[1] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[1][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[1][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[1][h][l][n] = new int[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //Definition du voisinage au centre de l'image
    int **neighImrRef = new int*[2], **neighImcRef = new int*[2];
    for (r = 0 ; r < 2; r++)
    {
        neighImrRef[r] = new int[nbNeigh];
        neighImcRef[r] = new int[nbNeigh];
    }
    int indNeigh = -derivativeMaskHalfSize;
    for (c = 0, k = 0 ; c < (nbNeigh+1); c++, indNeigh++)
        if(indNeigh != 0)
        {
            neighImrRef[0][k] = indNeigh;
            neighImrRef[1][k] = 0;

            neighImcRef[0][k] = 0;
            neighImcRef[1][k] = indNeigh;
            k++;
        }

    vpColVector XsCentre(3), uRot(3);
    vpColVector Xs(3);
    double theta;
    double x, y, u, v;
    bool projOK;
    XsCentre[2] = 1.;
    for (r = di ; r < nbri; r++)
        for (c = dj ; c < nbrj; c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            projOK = cam.image2Sphere(x, y, Xs[0], Xs[1], Xs[2]);

            if(projOK)
            {
                uRot = vpColVector::cross(XsCentre, Xs);
                uRot.normalize();
                theta = acos(vpColVector::dotProd(XsCentre, Xs));

                vpThetaUVector TU(theta*uRot[0], theta*uRot[1], theta*uRot[2]);
                vpRotationMatrix pRc = vpRotationMatrix(TU);

                for(j = 0 ; j < nbNeigh ; j++)
                {
                    v = v0 + neighImrRef[0][j];
                    u = u0 + neighImrRef[1][j];

                    vpPixelMeterConversion::convertPoint(cam, u, v, x, y);
                    cam.image2Sphere(x, y, Xs);

                    Xs = pRc * Xs;

                    cam.sphere2Image(Xs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[1][r][c][1][j] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[1][r][c][0][j] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }

                for(j = 0 ; j < nbNeigh ; j++)
                {
                    v = v0 + neighImcRef[0][j];
                    u = u0 + neighImcRef[1][j];
                    vpPixelMeterConversion::convertPoint(cam, u, v, x, y);

                    cam.image2Sphere(x, y, Xs);

                    Xs = pRc * Xs;

                    cam.sphere2Image(Xs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[0][r][c][1][j] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[0][r][c][0][j] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh[0][r][c][1][j] = Neigh[0][r][c][0][j] = Neigh[1][r][c][1][j] = Neigh[1][r][c][0][j] = 0;
                }
            }
        }
}

//Neigh[0] <=> VoisPhi
//Neigh[1] <=> VoisTheta

void
CFeatureLuminanceOmni::createNeighPhiThetaSpher()
{
    if(isNeighSet)
        return;

    isNeighSet = true;

    //Voisinage 2D
    Neigh = new int****[2];

    int nba = 2, r, c, p, h, l, n;

    Neigh[0] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[0][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[0][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[0][h][l][n] = new int[nbNeigh];
            }
        }
    }

    Neigh[1] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[1][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[1][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[1][h][l][n] = new int[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = 0;

    double x, y, theta, phi, vtheta, vphi, dssphi, u, v, sphi;
    int i, j, dtheta, dphi;
    for (r = di ; r < (di+nbri); r++)
        for (c = dj ; c < (dj+nbrj); c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            if(cam.image2SphereAngles(x, y, theta, phi) && (phi != 0.0) && (fabs(phi) != M_PI))
                //if( (phi > -300) && (phi != 0.0) && (phi != M_PI) )
            {
                // Voisinage "linÃ©aire" en Theta
                sphi = sin(phi);
                vphi = phi;
                dssphi = deltaPSSampling/sphi;
                vtheta = theta - dssphi*nbNeigh/2.0;
                i = 0;
                for (dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, /*vtheta += dssphi,*/ i++)
                {
                    vtheta = theta + deltaPSSampling*dtheta;///sphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[1][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[1][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
                vtheta = theta + dssphi;
                i = nbNeigh/2;
                for (dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, /*vtheta += dssphi,*/ i++)
                {
                    vtheta = theta + deltaPSSampling*dtheta;///sphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[1][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[1][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }

                // Voisinage "linÃ©aire" en Phi
                vtheta = theta;
                vphi = phi - deltaPSSampling*nbNeigh/2.0;
                i = 0;
                for (dphi = -nbNeigh/2 ; dphi < 0; dphi++, /*vphi += delta,*/ i++)
                {
                    vphi = phi + deltaPSSampling*dphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[0][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[0][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
                vphi = phi + deltaPSSampling;
                i = nbNeigh/2;
                for (dphi = 1 ; dphi <= nbNeigh/2; dphi++, /*vphi += delta,*/ i++)
                {
                    vphi = phi + deltaPSSampling*dphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[0][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[0][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh[0][r][c][1][j] = Neigh[0][r][c][0][j] = Neigh[1][r][c][1][j] = Neigh[1][r][c][0][j] = 0;
                }
            }
        }
}

//Neigh[0] <=> VoisXs
//Neigh[1] <=> VoisYs
//Neigh[2] <=> VoisZs

void
CFeatureLuminanceOmni::createNeighXsYsZsSpher()
{
    if(isNeighSet)
        return;

    isNeighSet = true;

    //Voisinage 3D
    Neigh = new int****[3];

    int nba = 2, r, c, p, h, l, n, j;

    Neigh[0] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[0][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[0][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[0][h][l][n] = new int[nbNeigh];
            }
        }
    }

    Neigh[1] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[1][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[1][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[1][h][l][n] = new int[nbNeigh];
            }
        }
    }

    Neigh[2] = new int***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh[2][h] = new int**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh[2][h][l] = new int*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh[2][h][l][n] = new int[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = Neigh[2][r][c][0][p] = Neigh[2][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = Neigh[2][r][c][0][p] = Neigh[2][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = Neigh[2][r][c][0][p] = Neigh[2][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh[0][r][c][0][p] = Neigh[0][r][c][1][p] = Neigh[1][r][c][0][p] = Neigh[1][r][c][1][p] = Neigh[2][r][c][0][p] = Neigh[2][r][c][1][p] = 0;


    double x, y, Xs, Ys, Zs, u, v, vXs, vYs, vZs;
    int i;
    for (r = di ; r < (di+nbri); r++)
        for (c = dj ; c < (dj+nbrj); c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            if(cam.image2Sphere(x, y, Xs, Ys, Zs))
            {
                // Voisinage "linÃ©aire" en Xs
                vXs = Xs - deltaCSSampling*nbNeigh/2.0;
                vYs = Ys;
                vZs = Zs;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vXs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[0][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[0][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
                vXs = Xs + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vXs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[0][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[0][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }

                // Voisinage "linÃ©aire" en Ys
                vXs = Xs;
                vYs = Ys - deltaCSSampling*nbNeigh/2.0;
                vZs = Zs;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vYs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[1][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[1][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
                vYs = Ys + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vYs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[1][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[1][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }


                // Voisinage "linÃ©aire" en Zs
                vXs = Xs;
                vYs = Ys;
                vZs = Zs - deltaCSSampling*nbNeigh/2.0;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vZs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[2][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[2][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }
                vZs = Zs + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vZs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh[2][r][c][1][i] = vpMath::round((u >= 0)?((u < imWidth)?u:(imWidth-1)):0);
                    Neigh[2][r][c][0][i] = vpMath::round((v >= 0)?((v < imHeight)?v:(imHeight-1)):0);
                }

            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh[0][r][c][0][j] = Neigh[0][r][c][1][j] = Neigh[1][r][c][0][j] = Neigh[1][r][c][1][j] = Neigh[2][r][c][0][j] = Neigh[2][r][c][1][j] = 0;
                }
            }
        }
}

void
CFeatureLuminanceOmni::deleteNeigh()
{
    if(!isNeighSet)
        return;

    isNeighSet = false;

    int nba = 2;

    for(int numDim = 0 ; numDim < nbDim; numDim++)
    {
        for(int h = 0 ; h < imHeight ; h++)
        {
            for(int l = 0 ; l < imWidth ; l++)
            {
                for(int n = 0 ; n < 2 ; n++)
                {
                    delete [] Neigh[numDim][h][l][n];
                }
                delete [] Neigh[numDim][h][l];
            }
            delete [] Neigh[numDim][h];
        }
        delete [] Neigh[numDim];
    }
}

//float neighborhoods in image plane

//Neigh_r[0] <=> VoisX
//Neigh_r[1] <=> VoisY

void
CFeatureLuminanceOmni::createNeighXYAdapt_r()
{
    if(isNeighSet_r)
        return;

    isNeighSet_r = true;

    //Voisinage 2D
    Neigh_r = new float****[2];

    int nba = 2, h, l, n, r, c, p, j, k;

    double u0 = cam.get_u0(), v0 = cam.get_v0();

    Neigh_r[0] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[0][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[0][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[0][h][l][n] = new float[nbNeigh];
            }
        }
    }

    Neigh_r[1] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[1][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[1][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[1][h][l][n] = new float[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //Definition du voisinage au centre de l'image
    float **neighImrRef = new float*[2], **neighImcRef = new float*[2];
    for (r = 0 ; r < 2; r++)
    {
        neighImrRef[r] = new float[nbNeigh];
        neighImcRef[r] = new float[nbNeigh];
    }
    float indNeigh = -derivativeMaskHalfSize;
    for (c = 0, k = 0 ; c < (nbNeigh+1); c++, indNeigh++)
        if(indNeigh != 0)
        {
            neighImrRef[0][k] = indNeigh;
            neighImrRef[1][k] = 0;

            neighImcRef[0][k] = 0;
            neighImcRef[1][k] = indNeigh;
            k++;
        }

    vpColVector XsCentre(3), uRot(3);
    vpColVector Xs(3);
    double theta;
    double x, y, u, v;
    bool projOK;
    XsCentre[2] = 1.;
    for (r = di ; r < nbri; r++)
        for (c = dj ; c < nbrj; c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            projOK = cam.image2Sphere(x, y, Xs[0], Xs[1], Xs[2]);

            if(projOK)
            {
                uRot = vpColVector::cross(XsCentre, Xs);
                uRot.normalize();
                theta = acos(vpColVector::dotProd(XsCentre, Xs));

                vpThetaUVector TU(theta*uRot[0], theta*uRot[1], theta*uRot[2]);
                vpRotationMatrix pRc = vpRotationMatrix(TU);

                for(j = 0 ; j < nbNeigh ; j++)
                {
                    v = v0 + neighImrRef[0][j];
                    u = u0 + neighImrRef[1][j];

                    vpPixelMeterConversion::convertPoint(cam, u, v, x, y);
                    cam.image2Sphere(x, y, Xs);

                    Xs = pRc * Xs;

                    cam.sphere2Image(Xs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[1][r][c][1][j] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[1][r][c][0][j] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }

                for(j = 0 ; j < nbNeigh ; j++)
                {
                    v = v0 + neighImcRef[0][j];
                    u = u0 + neighImcRef[1][j];
                    vpPixelMeterConversion::convertPoint(cam, u, v, x, y);

                    cam.image2Sphere(x, y, Xs);

                    Xs = pRc * Xs;

                    cam.sphere2Image(Xs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[0][r][c][1][j] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[0][r][c][0][j] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh_r[0][r][c][1][j] = Neigh_r[0][r][c][0][j] = Neigh_r[1][r][c][1][j] = Neigh_r[1][r][c][0][j] = 0;
                }
            }
        }
}

//Neigh_r[0] <=> VoisPhi
//Neigh_r[1] <=> VoisTheta

void
CFeatureLuminanceOmni::createNeighPhiThetaSpher_r()
{
    if(isNeighSet_r)
        return;

    isNeighSet_r = true;

    //Voisinage 2D
    Neigh_r = new float****[2];

    int nba = 2, r, c, p, h, l, n;

    Neigh_r[0] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[0][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[0][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[0][h][l][n] = new float[nbNeigh];
            }
        }
    }

    Neigh_r[1] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[1][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[1][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[1][h][l][n] = new float[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = 0;

    double x, y, theta, phi, vtheta, vphi, dssphi, u, v, sphi;
    int i, j, dtheta, dphi;
    for (r = di ; r < (di+nbri); r++)
        for (c = dj ; c < (dj+nbrj); c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            if(cam.image2SphereAngles(x, y, theta, phi) && (phi != 0.0) && (fabs(phi) != M_PI))
                //if( (phi > -300) && (phi != 0.0) && (phi != M_PI) )
            {
                // Voisinage "linÃ©aire" en Theta
                sphi = sin(phi);
                vphi = phi;
                dssphi = deltaPSSampling/sphi;
                vtheta = theta - dssphi*nbNeigh/2.0;
                i = 0;
                for (dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, /*vtheta += dssphi,*/ i++)
                {
                    vtheta = theta + deltaPSSampling*dtheta;///sphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[1][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[1][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
                vtheta = theta + dssphi;
                i = nbNeigh/2;
                for (dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, /*vtheta += dssphi,*/ i++)
                {
                    vtheta = theta + deltaPSSampling*dtheta;///sphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[1][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[1][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }

                // Voisinage "linÃ©aire" en Phi
                vtheta = theta;
                vphi = phi - deltaPSSampling*nbNeigh/2.0;
                i = 0;
                for (dphi = -nbNeigh/2 ; dphi < 0; dphi++, /*vphi += delta,*/ i++)
                {
                    vphi = phi + deltaPSSampling*dphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[0][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[0][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
                vphi = phi + deltaPSSampling;
                i = nbNeigh/2;
                for (dphi = 1 ; dphi <= nbNeigh/2; dphi++, /*vphi += delta,*/ i++)
                {
                    vphi = phi + deltaPSSampling*dphi;
                    cam.sphereAngles2Image(vtheta, vphi, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[0][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[0][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh_r[0][r][c][1][j] = Neigh_r[0][r][c][0][j] = Neigh_r[1][r][c][1][j] = Neigh_r[1][r][c][0][j] = 0;
                }
            }
        }
}

//Neigh_r[0] <=> VoisXs
//Neigh_r[1] <=> VoisYs
//Neigh_r[2] <=> VoisZs

void
CFeatureLuminanceOmni::createNeighXsYsZsSpher_r()
{
    if(isNeighSet_r)
        return;

    isNeighSet_r = true;

    //Voisinage 3D
    Neigh_r = new float****[3];

    int nba = 2, r, c, p, h, l, n, j;

    Neigh_r[0] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[0][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[0][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[0][h][l][n] = new float[nbNeigh];
            }
        }
    }

    Neigh_r[1] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[1][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[1][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[1][h][l][n] = new float[nbNeigh];
            }
        }
    }

    Neigh_r[2] = new float***[imHeight];
    for(h = 0 ; h < imHeight ; h++)
    {
        Neigh_r[2][h] = new float**[imWidth];
        for(l = 0 ; l < imWidth ; l++)
        {
            Neigh_r[2][h][l] = new float*[nba];
            for(n = 0 ; n < nba ; n++)
            {
                Neigh_r[2][h][l][n] = new float[nbNeigh];
            }
        }
    }

    //mise à zéro des coordonnées des voisins qui ne seront pas utilisées (bords des images)
    //bord supérieur
    for (r = 0 ; r < di; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = Neigh_r[2][r][c][0][p] = Neigh_r[2][r][c][1][p] = 0;

    //bord gauche
    for (r = di ; r < imHeight; r++)
        for (c = 0 ; c < dj; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = Neigh_r[2][r][c][0][p] = Neigh_r[2][r][c][1][p] = 0;

    //bord droit
    for (r = di ; r < imHeight; r++)
        for (c = dj+nbrj ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = Neigh_r[2][r][c][0][p] = Neigh_r[2][r][c][1][p] = 0;

    //bord inférieur
    for (r = nbri ; r < imHeight; r++)
        for (c = 0 ; c < imWidth; c++)
            for (p = 0 ; p < nbNeigh; p++)
                Neigh_r[0][r][c][0][p] = Neigh_r[0][r][c][1][p] = Neigh_r[1][r][c][0][p] = Neigh_r[1][r][c][1][p] = Neigh_r[2][r][c][0][p] = Neigh_r[2][r][c][1][p] = 0;


    double x, y, Xs, Ys, Zs, u, v, vXs, vYs, vZs;
    int i;
    for (r = di ; r < (di+nbri); r++)
        for (c = dj ; c < (dj+nbrj); c++)
        {
            vpPixelMeterConversion::convertPoint(cam, (double)c, (double)r, x, y);
            if(cam.image2Sphere(x, y, Xs, Ys, Zs))
            {
                // Voisinage "linÃ©aire" en Xs
                vXs = Xs - deltaCSSampling*nbNeigh/2.0;
                vYs = Ys;
                vZs = Zs;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vXs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[0][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[0][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
                vXs = Xs + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vXs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[0][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[0][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }

                // Voisinage "linÃ©aire" en Ys
                vXs = Xs;
                vYs = Ys - deltaCSSampling*nbNeigh/2.0;
                vZs = Zs;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vYs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[1][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[1][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
                vYs = Ys + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vYs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[1][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[1][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }


                // Voisinage "linÃ©aire" en Zs
                vXs = Xs;
                vYs = Ys;
                vZs = Zs - deltaCSSampling*nbNeigh/2.0;
                i = 0;
                for (int dtheta = -nbNeigh/2 ; dtheta < 0; dtheta++, vZs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[2][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[2][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }
                vZs = Zs + deltaCSSampling;
                i = nbNeigh/2;
                for (int dtheta = 1 ; dtheta <= nbNeigh/2; dtheta++, vZs += deltaCSSampling, i++)
                {
                    cam.sphere2Image(vXs, vYs, vZs, x, y);
                    vpMeterPixelConversion::convertPoint(cam, x, y, u, v);
                    Neigh_r[2][r][c][1][i] = (u >= 0)?((u < imWidth)?u:(imWidth-1)):0;
                    Neigh_r[2][r][c][0][i] = (v >= 0)?((v < imHeight)?v:(imHeight-1)):0;
                }

            }
            else
            {
                for(j = 0 ; j < nbNeigh ; j++)
                {
                    Neigh_r[0][r][c][0][j] = Neigh_r[0][r][c][1][j] = Neigh_r[1][r][c][0][j] = Neigh_r[1][r][c][1][j] = Neigh_r[2][r][c][0][j] = Neigh_r[2][r][c][1][j] = 0;
                }
            }
        }
}

void
CFeatureLuminanceOmni::deleteNeigh_r()
{
    if(!isNeighSet_r)
        return;

    isNeighSet_r = false;

    int nba = 2;

    for(int numDim = 0 ; numDim < nbDim; numDim++)
    {
        for(int h = 0 ; h < imHeight ; h++)
        {
            for(int l = 0 ; l < imWidth ; l++)
            {
                for(int n = 0 ; n < 2 ; n++)
                {
                    delete [] Neigh_r[numDim][h][l][n];
                }
                delete [] Neigh_r[numDim][h][l];
            }
            delete [] Neigh_r[numDim][h];
        }
        delete [] Neigh_r[numDim];
    }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
