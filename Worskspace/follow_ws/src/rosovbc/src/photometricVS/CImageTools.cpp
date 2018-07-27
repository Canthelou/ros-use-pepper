/****************************************************************************
 *
 * July 2011
 *
 * Author:
 * Guillaume Caron
 * inspired from visp/vpImageFilter (Eric Marchand)
 *
 *****************************************************************************/

#include <photometricVS/CImageTools.h>

//#include <PhOVS/CImageTools.h>

/*!
  Compute the signed difference between the two images I1 and I2 for
  visualization issue : Idiff = I1-I2

  - pixels with a null difference are set to 128.
  - A negative difference implies a pixel value < 128
  - A positive difference implies a pixel value > 128
  
  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference.
*/
void CImageTools::imageDifference(vpImage<unsigned char> &I1, 
                                  vpImage<unsigned char> &I2,
                                  vpImage<unsigned char> &Idiff)
{
    if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth()))
    {
        throw (vpException(vpException::dimensionError, "The two images have not the same size"));
    }

    if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
        Idiff.resize(I1.getHeight(), I1.getWidth());

    int n = I1.getHeight() * I1.getWidth() ;
    int diff ;
    unsigned char *pt_I1 = I1.bitmap, *pt_I2 = I2.bitmap, *pt_Idiff = Idiff.bitmap;
    for (int b = 0; b < n ; b++, pt_I1++, pt_I2++, pt_Idiff++)
    {
        diff = *pt_I1 - *pt_I2 + 128;
        *pt_Idiff = (unsigned char) (vpMath::maximum(vpMath::minimum(diff, 255), 0));
    }
}

/*!
  Compute the Zero-mean Normalized Correlation between I1 and I2
  
  \param I1 : The first image.
  \param I2 : The second image.
  \return : ZNCC of I1 and I2
*/
double CImageTools::imageZNCC(vpImage<unsigned char> &I1, 
                              vpImage<unsigned char> &I2)
{
    if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth()))
    {
        throw (vpException(vpException::dimensionError, "The two images have not the same size"));
    }

    int n = I1.getHeight() * I1.getWidth() ;
    double meanI1 = 0., meanI2 = 0.;
    unsigned char *pt_I1 = I1.bitmap, *pt_I2 = I2.bitmap;
    vpColVector vI1(n), vI2(n);
    double *pt_vI1 = vI1.data, *pt_vI2 = vI2.data;
    for (int b = 0; b < n ; b++, pt_I1++, pt_I2++, pt_vI1++, pt_vI2++)
    {
        meanI1 += *pt_I1;
        *pt_vI1 = *pt_I1;
        meanI2 += *pt_I2;
        *pt_vI2 = *pt_I2;
    }

    meanI1 /= (double)n;
    meanI2 /= (double)n;

    vI1 = *vI1.data - meanI1;
    vI1.normalize();
    vI2 = *vI2.data - meanI2;
    vI2.normalize();

    return vpColVector::dotProd(vI1,vI2);
}

/*!
  Compute the Zero-mean Normalized Correlation between I1 and I2
  
  \param I1 : The first image.
  \param I2 : The second image (already center and normalized)
  \return : ZNCC of I1 and I2
*/
double CImageTools::imageZNCC(vpImage<unsigned char> &I1, 
                              vpImage<double> &I2)
{
    if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth()))
    {
        throw (vpException(vpException::dimensionError, "The two images have not the same size"));
    }

    int n = I1.getHeight() * I1.getWidth() ;
    double meanI1 = 0., meanI2 = 0.;
    unsigned char *pt_I1 = I1.bitmap;
    vpColVector vI1(n), vI2(n);
    double *pt_vI1 = vI1.data;
    for (int b = 0; b < n ; b++, pt_I1++, pt_vI1++)
    {
        meanI1 += *pt_I1;
        *pt_vI1 = *pt_I1;
    }

    meanI1 /= (double)n;

    vI1 = *vI1.data - meanI1;
    vI1.normalize();

    memcpy(vI2.data, I2.bitmap, n*sizeof(double));

    return vpColVector::dotProd(vI1,vI2);
}

/*!
  Compute the Zero-mean Normalized Correlation between I1 and I2
  
  \param I1 : The first image.
  \param vI : A vector of the second image (row major, already center and normalized)
  \return : ZNCC of I1 and I2
*/
double CImageTools::imageZNCC(vpImage<unsigned char> &I1, 
                              vpColVector &vI2zn)
{
    int n = I1.getHeight() * I1.getWidth() ;
    if (vI2zn.getRows() != n)
    {
        throw (vpException(vpException::dimensionError, "The two images have not the same size"));
    }

    vpColVector vI1zn(n);
    CImageTools::imageZN(I1, vI1zn);

    return vpColVector::dotProd(vI1zn,vI2zn);
}

/*!
  Compute the ZNCC between I1 and I2

  \param I1 : Source image
  \param I2 : output zero mean and normalized image
*/
void CImageTools::imageZN(vpImage<unsigned char> &I1,
                          vpImage<unsigned char> &dst_im)
{

    int n = I1.getHeight() * I1.getWidth() ;
    vpColVector vI2(n);

    CImageTools::imageZN(I1, vI2);

    vpImage<double> I2(I1.getHeight(), I1.getWidth());
    memcpy(I2.bitmap, vI2.data, n*sizeof(double));

//    dst_im.resize(I1.getHeight(), I1.getWidth());

    vpImageConvert::convert(I2, dst_im);

    return;

}



/*!
  Compute the ZNCC between I1 and I2
  
  \param I1 : Source image
  \param I2 : output zero mean and normalized image
*/
void CImageTools::imageZN(vpImage<unsigned char> &I1, 
                          vpImage<double> &I2)
{
    int n = I1.getHeight() * I1.getWidth() ;
    vpColVector vI2(n);

    CImageTools::imageZN(I1, vI2);

    I2.resize(I1.getHeight(), I2.getWidth());
    memcpy(I2.bitmap, vI2.data, n*sizeof(double));
}




/*!
  Compute the ZNCC between I1 and I2
  
  \param I1 : Source image
  \param I2 : output zero mean and normalized image
*/
void CImageTools::imageZN(vpImage<unsigned char> &I1, 
                          vpColVector &V)
{
    int n = I1.getHeight() * I1.getWidth() ;
    double meanI1 = 0.;
    unsigned char *pt_I1 = I1.bitmap;

    V.resize(n);

    double *pt_V = V.data;
    for (int b = 0; b < n ; b++, pt_I1++, pt_V++)
    {
        meanI1 += *pt_I1;
        *pt_V = *pt_I1;
    }

    meanI1 /= (double)n;

    V = *V.data - meanI1;
    V.normalize();
}



