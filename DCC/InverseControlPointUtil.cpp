// Implementation for Distortion Coefficient Calibration class

#include "stdafx.h"
#include "InverseControlPointUtil.h"

#include "math.h"

#include "Warning.h"
#include "MathUtil.h"
#include "CDistortFunction.h"
#include "CInverseControlPointFunction.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////////
/// Inverse Control Point Calculation Class
//////////////////////////////////////////////////////////////////////////
void InverseControlPointUtil::calculate(const PicData* pPicData, const SimplexVertex* pExteriorParams, 
                                        const SimplexVertex* pInteriorParams, const Matrix* pMatrix, 
                                        int dataN, ObjectData* pInvCtrData)
{
   if (!pPicData || !pExteriorParams || !pInteriorParams || !pMatrix || !pInvCtrData)
   {
      DBG_WARN("Null pointer in InverseControlPointUtil::calculate");
      return;
   }

   int pointN = 0;
   for (int ii = 0; ii < pPicData[0].getPointNum(); ++ii)
   {
      int index = pPicData[0].getPoint(ii).getIndex();
      bool isMatch = true;
      for (int jj = 1; jj < dataN; ++jj)
      {
         if (pPicData[jj].find(index) == -1)
         {
            isMatch = false;
            break;
         }
      }
      if (isMatch)
         ++ pointN;
   }
   if (!pointN)
      return;
   pInvCtrData->setPointNum(pointN);

   PicControlPoint* pPoints = new PicControlPoint[dataN];
   int* pPos = new int[dataN];
   for (int ii = 0, jj = 0; ii < pPicData[0].getPointNum(); ++ii)
   {
      pPoints[0] = pPicData[0].getPoint(ii);
      pPos[0] = ii;
      int index = pPoints[0].getIndex();
      bool isMatch = true;
      for (int kk = 1; kk < dataN; ++kk)
      {
         pPos[kk] = pPicData[kk].find(index);
         if (pPos[kk] == -1)
         {
            isMatch = false;
            break;
         }
      }
      if (!isMatch)
         continue;

      for (int kk = 1; kk < dataN; ++kk)
      {
         pPoints[kk] = pPicData[kk].getPoint(pPos[kk]);
      }

      ObjControlPoint objPnt = getInvCtrPnt(pPoints,pExteriorParams,pInteriorParams,pMatrix,dataN);
      pInvCtrData->setPoint(jj,objPnt);
      ++ jj;
   }

   if (pPoints)   delete[] pPoints;
   if (pPos) delete[] pPos;
}

SimplexVertex 
InverseControlPointUtil::initInvCtrPnt(const PicControlPoint* pPoints, const SimplexVertex* pExteriorParams, 
                                       const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
                                       int dataN)
{
   SimplexVertex distortParams(4);
   ImagePoint origin;

   double u, v, f, Xs, Ys, Zs;
   double a1,a2,a3,b1,b2,b3,c1,c2,c3;

   Matrix mA(2*dataN,3);
   Matrix mL(2*dataN,1);

   for (int ii = 0; ii < dataN; ++ii)
   {
      ImagePoint point = pPoints[ii];
      const SimplexVertex& exteriorParams = pExteriorParams[ii];
      const SimplexVertex& interiorParams = pInteriorParams[ii];
      const Matrix& rotMatrix = pMatrix[ii];

      f = interiorParams(6); Xs = exteriorParams(0);Ys = exteriorParams(1); Zs = exteriorParams(2);
      a1 = rotMatrix(0,0); a2 = rotMatrix(0,1); a3 = rotMatrix(0,2);
      b1 = rotMatrix(1,0); b2 = rotMatrix(1,1); b3 = rotMatrix(1,2);
      c1 = rotMatrix(2,0); c2 = rotMatrix(2,1); c3 = rotMatrix(2,2);
      origin.setXPos(interiorParams(0)); origin.setYPos(interiorParams(1));
      for (int jj = 0; jj < 4; ++jj)
      {
         distortParams(jj) = interiorParams(jj+2);
      }
      ImagePoint undistortPnt;
      CFUnDistort func(distortParams,point,origin);
      if (!point.undistort(func,&undistortPnt))
      {
         undistortPnt = point;
      }
      undistortPnt -= origin;
      u = undistortPnt.getXPos(); v = undistortPnt.getYPos();

      mA(2*ii,0) = a1*f + a3*u; mA(2*ii,1) = b1*f + b3*u; mA(2*ii,2) = c1*f + c3*u;
      mA(2*ii+1,0) = a2*f + a3*v; mA(2*ii+1,1) = b2*f + b3*v; mA(2*ii+1,2) = c2*f + c3*v;
      mL(2*ii,0) = (a1*f + a3*u)*Xs + (b1*f + b3*u)*Ys + (c1*f + c3*u)*Zs;
      mL(2*ii+1,0) = (a2*f + a3*v)*Xs + (b2*f + b3*v)*Ys + (c2*f + c3*v)*Zs;
   }

   return LinearEquations::solve(mA,mL);
}

ObjControlPoint 
InverseControlPointUtil::getInvCtrPnt(const PicControlPoint* pPoints, const SimplexVertex* pExteriorParams, 
                                      const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
                                      int dataN)
{
   SimplexVertex vx = initInvCtrPnt(pPoints,pExteriorParams,pInteriorParams,pMatrix,dataN);

   if (vx.getDim() != 3)
   {
      DBG_WARN("Wrong dimension of vector in InverseControlPointUtil::getInvCtrPnt");
      return ObjControlPoint();
   }

   CInverseControlPoint func(pPoints,pExteriorParams,pInteriorParams,pMatrix,dataN);
   Newton::solve(vx,func);

   return ObjControlPoint(pPoints[0].getIndex(),vx(0),vx(1),vx(2));
}