#include "stdafx.h"
#include "CPhotoCalibrationFunction.h"

#include "Warning.h"
#include "math.h"
#include "CDistortFunction.h"

#define EPS 1.0e-4		// Approximate square root of the machine precision

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////////
/// Implementation of CPhotoCalibration
//////////////////////////////////////////////////////////////////////////
CPhotoCalibration::CPhotoCalibration(const PicData &picData, const ObjectData &objectData, 
                                     const SimplexVertex &distortParams, const ImagePoint &origin,
                                     double f)
                                     : m_picData(picData)
                                     , m_objectData(objectData)
                                     , m_distortParams(distortParams)
                                     , m_origin(origin)
                                     , m_f(f)
{

}

SimplexVertex CPhotoCalibration::operator ()(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   SimplexVertex vy(2*mm);

   double k1 = m_distortParams(0), k2 = m_distortParams(1), p1 = m_distortParams(2), p2 = m_distortParams(3);
   double x0 = m_origin.getXPos(), y0 = m_origin.getYPos();

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);
      double u = -m_f*XX/ZZ, v = -m_f*YY/ZZ;
      double u2 = u*u, v2 = v*v, uv2 = u*u+v*v, uv = u*v;
      double deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      double deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);

      xP -= x0 + deltaU;
      yP -= y0 + deltaV;

      vy(row) = -xP - m_f*XX/ZZ;
      vy(row+1) = -yP - m_f*YY/ZZ;

      row +=2;
   }

   return vy;
}

Matrix CPhotoCalibration::Jacobian(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   const int nn = vx.getDim();
   Matrix mA(2*mm,nn);

   double k1 = m_distortParams(0), k2 = m_distortParams(1), p1 = m_distortParams(2), p2 = m_distortParams(3);
   double x0 = m_origin.getXPos(), y0 = m_origin.getYPos();

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   SimplexVertex rA1(nn); // a11, a12, a13, a14, a15, a16, a17
   SimplexVertex rA2(nn); // a21, a22, a23, a24, a25, a26, a27

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);
      double u = -m_f*XX/ZZ, v = -m_f*YY/ZZ;
      double u2 = u*u, v2 = v*v, uv2 = u*u+v*v, uv = u*v;
      double deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      double deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);

      xP -= x0 + deltaU;
      yP -= y0 + deltaV;

      rA1(0) = (a1*m_f+a3*xP)/ZZ; rA1(1) = (b1*m_f+b3*xP)/ZZ; rA1(2) = (c1*m_f+c3*xP)/ZZ;
      rA1(3) = yP*sin(omiga) - (xP*(xP*cos(kapa) - yP*sin(kapa))/m_f + m_f*cos(kapa))*cos(omiga);
      rA1(4) = -m_f*sin(kapa) - xP*(xP*sin(kapa) + yP*cos(kapa))/m_f;
      rA1(5) = yP;

      rA2(0) = (a2*m_f+a3*yP)/ZZ; rA2(1) = (b2*m_f+b3*yP)/ZZ; rA2(2) = (c2*m_f+c3*yP)/ZZ; 
      rA2(3) = -xP*sin(omiga) - (yP*(xP*cos(kapa) - yP*sin(kapa))/m_f - m_f*sin(kapa))*cos(omiga);
      rA2(4) = -m_f*cos(kapa) - yP*(xP*sin(kapa) + yP*cos(kapa))/m_f;
      rA2(5) = -xP;

      mA.setRowVec(row,rA1);
      mA.setRowVec(row+1,rA2);

      row +=2;
   }

   return mA;
}


//////////////////////////////////////////////////////////////////////////
/// Implementation of CPhotoCalibrationAll
//////////////////////////////////////////////////////////////////////////
CPhotoCalibrationAll::CPhotoCalibrationAll(const PicData& picData, const ObjectData& objectData)
: m_picData(picData)
, m_objectData(objectData)
{

}

SimplexVertex CPhotoCalibrationAll::operator ()(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   SimplexVertex vy(2*mm);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);
   double f = vx(6), x0 = vx(7), y0 = vx(8);
   double k1 = vx(9), k2 = vx(10), p1 = vx(11), p2 = vx(12);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);
      double u = -f*XX/ZZ, v = -f*YY/ZZ;
      double u2 = u*u, v2 = v*v, uv2 = u*u+v*v, uv = u*v;
      double deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      double deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);

      xP -= x0 + deltaU;
      yP -= y0 + deltaV;

      vy(row) = -xP - f*XX/ZZ;
      vy(row+1) = -yP - f*YY/ZZ;

      row +=2;
   }

   return vy;
}

Matrix CPhotoCalibrationAll::Jacobian(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   const int nn = vx.getDim();
   Matrix mA(2*mm,nn);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);
   double f = vx(6), x0 = vx(7), y0 = vx(8);
   double k1 = vx(9), k2 = vx(10), p1 = vx(11), p2 = vx(12);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   SimplexVertex rA1(nn); // a11, a12, a13, a14, a15, a16, a17, a18, a19, a1A, a1B, a1C, a1D
   SimplexVertex rA2(nn); // a21, a22, a23, a24, a25, a26, a27, a28, a29, a2A, a2B, a2C, a2D

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);
      double u = -f*XX/ZZ, v = -f*YY/ZZ;
      double u2 = u*u, v2 = v*v, uv2 = u*u+v*v, uv = u*v;
      double deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      double deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);

      xP -= x0 + deltaU;
      yP -= y0 + deltaV;

      rA1(0) = (a1*f+a3*xP)/ZZ; rA1(1) = (b1*f+b3*xP)/ZZ; rA1(2) = (c1*f+c3*xP)/ZZ;
      rA1(3) = yP*sin(omiga) - (xP*(xP*cos(kapa) - yP*sin(kapa))/f + f*cos(kapa))*cos(omiga);
      rA1(4) = -f*sin(kapa) - xP*(xP*sin(kapa) + yP*cos(kapa))/f;
      rA1(5) = yP; rA1(6) = xP/f; rA1(7) = 1; rA1(8) = 0;
      rA1(9) = u*uv2; rA1(10) = u*uv2*uv2; rA1(11) = 3*u2+v2; rA1(12) = 2*uv;

      rA2(0) = (a2*f+a3*yP)/ZZ; rA2(1) = (b2*f+b3*yP)/ZZ; rA2(2) = (c2*f+c3*yP)/ZZ; 
      rA2(3) = -xP*sin(omiga) - (yP*(xP*cos(kapa) - yP*sin(kapa))/f - f*sin(kapa))*cos(omiga);
      rA2(4) = -f*cos(kapa) - yP*(xP*sin(kapa) + yP*cos(kapa))/f;
      rA2(5) = -xP; rA2(6) = yP/f; rA2(7) = 0; rA2(8) = 1;
      rA2(9) = v*uv2; rA2(10) = v*uv2*uv2; rA2(11) = 2*uv; rA2(12) = u2+3*v2;

      mA.setRowVec(row,rA1);
      mA.setRowVec(row+1,rA2);

      row +=2;
   }

   return mA;
}

//////////////////////////////////////////////////////////////////////////
/// Implementation of CPhotoCalibrationWithoutDistort
//////////////////////////////////////////////////////////////////////////
CPhotoCalibrationWithoutDistort::CPhotoCalibrationWithoutDistort(const PicData& picData, const ObjectData& objectData)
: m_picData(picData)
, m_objectData(objectData)
{

}

SimplexVertex CPhotoCalibrationWithoutDistort::operator ()(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   SimplexVertex vy(2*mm);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);
   double f = vx(6), x0 = vx(7), y0 = vx(8);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);

      xP -= x0;
      yP -= y0;

      vy(row) = -xP - f*XX/ZZ;
      vy(row+1) = -yP - f*YY/ZZ;

      row +=2;
   }

   return vy;
}

Matrix CPhotoCalibrationWithoutDistort::Jacobian(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   const int nn = vx.getDim();
   Matrix mA(2*mm,nn);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);
   double fi = vx(3), omiga = vx(4), kapa = vx(5);
   double f = vx(6), x0 = vx(7), y0 = vx(8);

   Matrix rotMatrix(MatrixUtil::generateRotateMatrix(fi,omiga,kapa));
   double a1 = rotMatrix(0,0), a2 = rotMatrix(0,1), a3 = rotMatrix(0,2);
   double b1 = rotMatrix(1,0), b2 = rotMatrix(1,1), b3 = rotMatrix(1,2);
   double c1 = rotMatrix(2,0), c2 = rotMatrix(2,1), c3 = rotMatrix(2,2);

   SimplexVertex rA1(nn); // a11, a12, a13, a14, a15, a16, a17, a18, a19
   SimplexVertex rA2(nn); // a21, a22, a23, a24, a25, a26, a27, a28, a29

   int row = 0;
   for (int ii = 0, jj; ii < m_picData.getPointNum(); ++ii)
   {
      const PicControlPoint& picPoint = m_picData.getPoint(ii);
      if ( (jj = m_objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = m_objectData.getPoint(jj);
      double xP = picPoint.getXPos(), yP = picPoint.getYPos();
      double xW = objPoint.getXPos(), yW = objPoint.getYPos(), zW = objPoint.getZPos();
      double XX = a1*(xW-Xs) + b1*(yW-Ys) + c1*(zW-Zs);
      double YY = a2*(xW-Xs) + b2*(yW-Ys) + c2*(zW-Zs);
      double ZZ = a3*(xW-Xs) + b3*(yW-Ys) + c3*(zW-Zs);

      xP -= x0;
      yP -= y0;

      rA1(0) = (a1*f+a3*xP)/ZZ; rA1(1) = (b1*f+b3*xP)/ZZ; rA1(2) = (c1*f+c3*xP)/ZZ;
      rA1(3) = yP*sin(omiga) - (xP*(xP*cos(kapa) - yP*sin(kapa))/f + f*cos(kapa))*cos(omiga);
      rA1(4) = -f*sin(kapa) - xP*(xP*sin(kapa) + yP*cos(kapa))/f;
      rA1(5) = yP; rA1(6) = xP/f; rA1(7) = 1; rA1(8) = 0;

      rA2(0) = (a2*f+a3*yP)/ZZ; rA2(1) = (b2*f+b3*yP)/ZZ; rA2(2) = (c2*f+c3*yP)/ZZ; 
      rA2(3) = -xP*sin(omiga) - (yP*(xP*cos(kapa) - yP*sin(kapa))/f - f*sin(kapa))*cos(omiga);
      rA2(4) = -f*cos(kapa) - yP*(xP*sin(kapa) + yP*cos(kapa))/f;
      rA2(5) = -xP; rA2(6) = yP/f; rA2(7) = 0; rA2(8) = 1;

      mA.setRowVec(row,rA1);
      mA.setRowVec(row+1,rA2);

      row +=2;
   }

   return mA;
}

//////////////////////////////////////////////////////////////////////////
/// Implementation of CPhotoCalibrationInitXYZFunc
//////////////////////////////////////////////////////////////////////////
CPhotoCalibrationInitXYZFunc::
CPhotoCalibrationInitXYZFunc(const PicData &picData, const ObjectData &objectData, 
                             const SimplexVertex &distortParams, const ImagePoint &origin,
                             double f)
                             : m_picData(picData)
                             , m_objectData(objectData)
                             , m_distortParams(distortParams)
                             , m_origin(origin)
                             , m_f(f)
{
}

SimplexVertex CPhotoCalibrationInitXYZFunc::operator ()(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   SimplexVertex vy(mm);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);

   for (int ii = 0; ii < mm; ++ii)
   {
      PicControlPoint picPoint[2];
      ObjControlPoint objPoint[2];
      ImagePoint undistortPnt[2];
      bool isMatch = true;
      for (int jj = 0; jj < 2; ++jj)
      {
         int ip = (ii+jj)%mm, jp;
         picPoint[jj] = m_picData.getPoint(ip);
         if ( (jp = m_objectData.find(picPoint[jj].getIndex())) == -1 )
         {
            isMatch = false;
            break;
         }
         objPoint[jj] = m_objectData.getPoint(jp);
         ImagePoint imgPoint(picPoint[jj]);
         CFUnDistort func(m_distortParams,imgPoint,m_origin);
         if (!imgPoint.undistort(func,&undistortPnt[jj]))
         {
            undistortPnt[jj] = imgPoint;
         }
         undistortPnt[jj] -= m_origin;
      }
      if (!isMatch)
         continue;

      double xi = undistortPnt[0].getXPos(), yi = undistortPnt[0].getYPos();
      double xj = undistortPnt[1].getXPos(), yj = undistortPnt[1].getYPos();
      double cij = (xi*xj + yi*yj + m_f*m_f) / (sqrt(xi*xi + yi*yi + m_f*m_f) * sqrt(xj*xj + yj*yj + m_f*m_f));

      double oXI = objPoint[0].getXPos() - Xs, oYI = objPoint[0].getYPos() - Ys, oZI = objPoint[0].getZPos() - Zs;
      double oXJ = objPoint[1].getXPos() - Xs, oYJ = objPoint[1].getYPos() - Ys, oZJ = objPoint[1].getZPos() - Zs;
      double oCIJ = (oXI*oXJ + oYI*oYJ + oZI*oZJ) / (sqrt(oXI*oXI + oYI*oYI + oZI*oZI) * sqrt(oXJ*oXJ + oYJ*oYJ + oZJ*oZJ));

      vy(ii) = oCIJ - cij;
   }

   return vy;
}

Matrix CPhotoCalibrationInitXYZFunc::Jacobian(const SimplexVertex &vx) const
{
   const int mm = m_picData.getPointNum();
   Matrix mA(mm,3);

   double Xs = vx(0), Ys = vx(1), Zs = vx(2);

   SimplexVertex vA(3);

   for (int ii = 0; ii < mm; ++ii)
   {
      PicControlPoint picPoint[2];
      ObjControlPoint objPoint[2];
      ImagePoint undistortPnt[2];
      bool isMatch = true;
      for (int jj = 0; jj < 2; ++jj)
      {
         int ip = (ii+jj)%mm, jp;
         picPoint[jj] = m_picData.getPoint(ip);
         if ( (jp = m_objectData.find(picPoint[jj].getIndex())) == -1 )
         {
            isMatch = false;
            break;
         }
         objPoint[jj] = m_objectData.getPoint(jp);
         ImagePoint imgPoint(picPoint[jj]);
         CFUnDistort func(m_distortParams,imgPoint,m_origin);
         if (!imgPoint.undistort(func,&undistortPnt[jj]))
         {
            undistortPnt[jj] = imgPoint;
         }
         undistortPnt[jj] -= m_origin;
      }
      if (!isMatch)
         continue;

      double xi = undistortPnt[0].getXPos(), yi = undistortPnt[0].getYPos();
      double xj = undistortPnt[1].getXPos(), yj = undistortPnt[1].getYPos();
      double cij = (xi*xj + yi*yj + m_f*m_f) / (sqrt(xi*xi + yi*yi + m_f*m_f) * sqrt(xj*xj + yj*yj + m_f*m_f));

      double oXI = objPoint[0].getXPos() - Xs, oYI = objPoint[0].getYPos() - Ys, oZI = objPoint[0].getZPos() - Zs;
      double oXJ = objPoint[1].getXPos() - Xs, oYJ = objPoint[1].getYPos() - Ys, oZJ = objPoint[1].getZPos() - Zs;
      double oSI = sqrt(oXI*oXI + oYI*oYI + oZI*oZI), oSJ = sqrt(oXJ*oXJ + oYJ*oYJ + oZJ*oZJ);

      vA(0) = -oXI * (1 - oSJ/oSI*cij) / (oSI*oSJ) - oXJ * (1 - oSI/oSJ*cij) / (oSI*oSJ);
      vA(1) = -oYI * (1 - oSJ/oSI*cij) / (oSI*oSJ) - oYJ * (1 - oSI/oSJ*cij) / (oSI*oSJ);
      vA(2) = -oZI * (1 - oSJ/oSI*cij) / (oSI*oSJ) - oZJ * (1 - oSI/oSJ*cij) / (oSI*oSJ);

      mA.setRowVec(ii,vA);
   }

   return mA;
}
