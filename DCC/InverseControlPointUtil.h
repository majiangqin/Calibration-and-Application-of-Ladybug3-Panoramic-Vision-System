#ifndef INVERSE_CONTROL_POINT_UTIL_H
#define INVERSE_CONTROL_POINT_UTIL_H

#pragma once

class SimplexVertex;
class PicData;
class ObjectData;
class Matrix;
class ObjControlPoint;
class PicControlPoint;

// Inverse Control Point Calculation
class InverseControlPointUtil
{
public:
   static void calculate(const PicData* pPicData, const SimplexVertex* pExteriorParams, 
      const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
      int dataN, ObjectData* pInvCtrData);

protected:
   static ObjControlPoint getInvCtrPnt(const PicControlPoint* pPoints, const SimplexVertex* pExteriorParams, 
      const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
      int dataN);

private:
   static SimplexVertex initInvCtrPnt(const PicControlPoint* pPoints, const SimplexVertex* pExteriorParams, 
      const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
      int dataN);
};

#endif