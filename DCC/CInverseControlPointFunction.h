#ifndef INVERSE_CONTROL_POINT_FUNCTION_H
#define INVERSE_CONTROL_POINT_FUNCTION_H

#include "CFunction.h"
#include "PicData.h"

#pragma once

class CInverseControlPoint : public CFunction
{
public:
   CInverseControlPoint(const PicControlPoint* pPoints, const SimplexVertex* pExteriorParams, 
      const SimplexVertex* pInteriorParams, const Matrix* pMatrix,
      int dataN);
   virtual ~CInverseControlPoint();

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

   virtual Matrix Jacobian(const SimplexVertex&) const;

private:
   PicControlPoint* m_pPoints;
   SimplexVertex* m_pExteriorParams; 
   SimplexVertex* m_pInteriorParams;
   Matrix* m_pMatrix;
   int m_dataN;
};

#endif