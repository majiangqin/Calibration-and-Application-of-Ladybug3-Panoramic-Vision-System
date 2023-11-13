#include "stdafx.h"
#include "CalibrationUtil.h"

#include "math.h"

#include "Warning.h"
#include "MathUtil.h"
#include "CDistortFunction.h"
#include "CPhotoCalibrationFunction.h"

#define HUGE   1.0e10
#define TOLD   1.0e-4

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////////
/// Photo Calibration Class
//////////////////////////////////////////////////////////////////////////
namespace
{
   SimplexVertex objControlPointVector(const ObjControlPoint& point)
   {
      SimplexVertex result(3);
      result(0) = point.getXPos();
      result(1) = point.getYPos();
      result(2) = point.getZPos();
      return result;
   }
}

void PhotoCalibrationUtil::initExteriorParams(const PicData& picData, const ObjectData& objectData, 
                                              const ImagePoint& origin, const SimplexVertex& distortParams,
                                              double f, SimplexVertex* pExtParams)
{
   DBG_WARN_AND_RETURN_VOID_UNLESS(pExtParams,"Null pointer in PhotoCalibrationUtil::initExteriorParams");
   pExtParams->setDim(7);
   pExtParams->setEntry(6,f);

   // Initiate Xs, Ys, Zs
   SimplexVertex XYZ(3);
   CPhotoCalibrationInitXYZFunc func(picData,objectData,distortParams,origin,f);
   Newton::solve(XYZ,func);
   pExtParams->setEntry(0,XYZ(0));
   pExtParams->setEntry(1,XYZ(1));
   pExtParams->setEntry(2,XYZ(2));

   // Initiate fi, omiga, kapa
   const int mm = picData.getPointNum();
   Matrix mA1(mm,3), mA2(mm,3), mA3(mm,3);
   Matrix mB1(mm,1), mB2(mm,1), mB3(mm,1);
   for (int ii = 0, jj; ii < mm; ++ii)
   {
      const PicControlPoint& picPoint = picData.getPoint(ii);
      if ( (jj = objectData.find(picPoint.getIndex())) == -1 )
         continue;
      const ObjControlPoint& objPoint = objectData.getPoint(jj);
      ImagePoint imgPnt(picPoint), undistortPnt;
      CFUnDistort undistortFunc(distortParams,imgPnt,origin);
      if (!imgPnt.undistort(undistortFunc,&undistortPnt))
      {
         undistortPnt = imgPnt;
      }
      undistortPnt -= origin;
      double xP = undistortPnt.getXPos(), yP = undistortPnt.getYPos();
      double xW = objPoint.getXPos() - XYZ(0), yW = objPoint.getYPos() - XYZ(1), zW = objPoint.getZPos() - XYZ(2);
      double sP = sqrt(xP*xP + yP*yP + f*f);
      double sW = sqrt(xW*xW + yW*yW + zW*zW);
      
      mA1(ii,0) = xW/sW; mA1(ii,1) = yW/sW; mA1(ii,2) = zW/sW; mB1(ii,0) = xP/sP;
      mA2(ii,0) = xW/sW; mA2(ii,1) = yW/sW; mA2(ii,2) = zW/sW; mB2(ii,0) = yP/sP;
      mA3(ii,0) = xW/sW; mA3(ii,1) = yW/sW; mA3(ii,2) = zW/sW; mB3(ii,0) = f/sP;
   }
   SimplexVertex a1b1c1 = LinearEquations::solve(mA1,mB1);
   SimplexVertex a2b2c2 = LinearEquations::solve(mA2,mB2);
   SimplexVertex a3b3c3 = LinearEquations::solve(mA3,mB3);

   Matrix rotMatrix(3,3);
   rotMatrix.setColVec(0,a1b1c1);
   rotMatrix.setColVec(1,a2b2c2);
   rotMatrix.setColVec(2,a3b3c3);

   double fi, omiga, kapa;
   MatrixUtil::computeRotateAngle(rotMatrix,&fi,&omiga,&kapa);
   pExtParams->setEntry(3,fi);
   pExtParams->setEntry(4,omiga);
   pExtParams->setEntry(5,kapa);
}

void PhotoCalibrationUtil::calculate(const PicData &picData, const ObjectData &objectData, 
                                     ImagePoint* pOrigin, SimplexVertex* pDistortParams, 
                                     SimplexVertex* pExteriorParams,
                                     bool withoutDistort)
{
   DBG_WARN_AND_RETURN_VOID_UNLESS(pOrigin&&pDistortParams&&pExteriorParams, "Null pointer in PhotoCalibrationUtil::calculate");

   if (withoutDistort)
   {
      for (int ii = 0; ii < pDistortParams->getDim(); ++ii)
         pDistortParams->setEntry(ii,0.0);
      const int external = pExteriorParams->getDim();
      SimplexVertex vx(external+2);
      vx.setSubSpace(0,external-1,*pExteriorParams);
      vx.setSubSpace(external,external+1,*pOrigin);
      CPhotoCalibrationWithoutDistort func(picData,objectData);
      Newton::solve(vx,func);
      *pExteriorParams = vx.getSubSpace(0,external-1);
      pOrigin->setXPos(vx(external)); pOrigin->setYPos(vx(external+1));
   }
   else
   {
      //CPhotoCalibration func1(picData,objectData,*pDistortParams,*pOrigin,pExteriorParams->getEntry(6));
      //SimplexVertex vx1(6);
      //vx1 = pExteriorParams->getSubSpace(0,5);
      //Newton::solve(vx1,func1);
      //pExteriorParams->setSubSpace(0,5,vx1);

      //const int external = pExteriorParams->getDim();
      //const int distort = pDistortParams->getDim();
      //SimplexVertex vx(external+distort+2);
      //vx.setSubSpace(0,external-1,*pExteriorParams);
      //vx.setSubSpace(external,external+1,*pOrigin);
      //vx.setSubSpace(external+2,external+distort+1,*pDistortParams);
      //CPhotoCalibrationAll func(picData,objectData);
      //Newton::solve(vx,func);
      //*pExteriorParams = vx.getSubSpace(0,external-1);
      //pOrigin->setXPos(vx(external)); pOrigin->setYPos(vx(external+1));
      //*pDistortParams = vx.getSubSpace(external+2,external+distort+1);
   }
}


//////////////////////////////////////////////////////////////////////////
/// Relative Calibration Class
//////////////////////////////////////////////////////////////////////////
void RelativeCalibrationUtil::calculate(const SimplexVertex &baseParams, const SimplexVertex &compareParams, 
                                        const Matrix &baseRotateMatrix, const Matrix &compareRotateMatrix, 
                                        SimplexVertex *pRelativeParams, Matrix *pRelativeRotateMatrix)
{
   DBG_WARN_AND_RETURN_VOID_UNLESS(pRelativeParams&&pRelativeRotateMatrix, "Null pointer in RelativeCalibrationUtil::calculate");
   *pRelativeRotateMatrix = compareRotateMatrix * Inv(baseRotateMatrix);
   Matrix mm(compareParams-baseParams);
   mm = baseRotateMatrix * mm;
   *pRelativeParams = mm;
}