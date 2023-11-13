#include "stdafx.h"
#include "CInverseControlPointFunction.h"

#include "Warning.h"
#include "math.h"

#define EPS 1.0e-4		// Approximate square root of the machine precision

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////////
/// Implementation of CInverseControlPoint
//////////////////////////////////////////////////////////////////////////
CInverseControlPoint::CInverseControlPoint(const PicControlPoint *pPoints, const SimplexVertex *pExteriorParams, 
                                           const SimplexVertex *pInteriorParams, const Matrix *pMatrix, int dataN)
                                           : m_dataN(dataN)
{
   m_pPoints = new PicControlPoint[dataN];
   m_pExteriorParams = new SimplexVertex[dataN];
   m_pInteriorParams = new SimplexVertex[dataN];
   m_pMatrix = new Matrix[dataN];
   if (!m_pMatrix || !m_pPoints || !m_pExteriorParams || !m_pInteriorParams)
   {
      DBG_WARN("Memory allocation failure in CInverseControlPoint::CInverseControlPoint");
      return;
   }
   if (!pMatrix || !pPoints || !pExteriorParams || !pInteriorParams)
   {
      DBG_WARN("Null vertex pointer in CInverseControlPoint::CInverseControlPoint");
      return;
   }
   for (int ii = 0; ii < dataN; ++ii)
   {
      m_pPoints[ii] = pPoints[ii];
      m_pInteriorParams[ii] = pInteriorParams[ii];
      m_pExteriorParams[ii] = pExteriorParams[ii];
      m_pMatrix[ii] = pMatrix[ii];
   }
}

CInverseControlPoint::~CInverseControlPoint()
{
   if (m_pPoints) delete[] m_pPoints;
   if (m_pExteriorParams) delete[] m_pExteriorParams;
   if (m_pInteriorParams) delete[] m_pInteriorParams;
   if (m_pMatrix) delete[] m_pMatrix;
}

SimplexVertex CInverseControlPoint::operator ()(const SimplexVertex &vx) const
{
   SimplexVertex vy(2*m_dataN);

   double x, y, x0, y0, f, Xs, Ys, Zs, k1, k2, p1, p2;
   double a1,a2,a3,b1,b2,b3,c1,c2,c3;
   double u, v, deltaU, deltaV;
   double Xa = vx(0), Ya = vx(1), Za = vx(2);

   for (int ii = 0; ii < m_dataN; ++ii)
   {
      ImagePoint point = m_pPoints[ii];
      const SimplexVertex& exteriorParams = m_pExteriorParams[ii];
      const SimplexVertex& interiorParams = m_pInteriorParams[ii];
      const Matrix& rotMatrix = m_pMatrix[ii];

      x = point.getXPos(); y = point.getYPos(); x0 = interiorParams(0); y0 = interiorParams(1);
      k1 = interiorParams(2); k2 = interiorParams(3); p1 = interiorParams(4); p2 = interiorParams(5);
      f = interiorParams(6); Xs = exteriorParams(0);Ys = exteriorParams(1); Zs = exteriorParams(2);
      a1 = rotMatrix(0,0); a2 = rotMatrix(0,1); a3 = rotMatrix(0,2);
      b1 = rotMatrix(1,0); b2 = rotMatrix(1,1); b3 = rotMatrix(1,2);
      c1 = rotMatrix(2,0); c2 = rotMatrix(2,1); c3 = rotMatrix(2,2);

      u = -f*(a1*(Xa-Xs)+b1*(Ya-Ys)+c1*(Za-Zs))/(a3*(Xa-Xs)+b3*(Ya-Ys)+c3*(Za-Zs)); 
      v = -f*(a2*(Xa-Xs)+b2*(Ya-Ys)+c2*(Za-Zs))/(a3*(Xa-Xs)+b3*(Ya-Ys)+c3*(Za-Zs));
      double uv2 = u*u+v*v, u2 = u*u, v2 = v*v, uv = u*v;
      deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);
      u = x - x0 - deltaU; v = y - y0 - deltaV;

      vy(2*ii) = (a1*f + a3*u)*(Xs - Xa) + (b1*f + b3*u)*(Ys - Ya) + (c1*f + c3*u)*(Zs - Za);
      vy(2*ii+1) = (a2*f + a3*v)*(Xs - Xa) + (b2*f + b3*v)*(Ys - Ya) + (c2*f + c3*v)*(Zs - Za);
   }

   return vy;
}

Matrix CInverseControlPoint::Jacobian(const SimplexVertex &vx) const
{
   Matrix mA(2*m_dataN,3);

   double x, y, x0, y0, f, Xs, Ys, Zs, k1, k2, p1, p2;
   double a1,a2,a3,b1,b2,b3,c1,c2,c3;
   double u, v, deltaU, deltaV;
   double Xa = vx(0), Ya = vx(1), Za = vx(2);

   for (int ii = 0; ii < m_dataN; ++ii)
   {
      ImagePoint point = m_pPoints[ii];
      const SimplexVertex& exteriorParams = m_pExteriorParams[ii];
      const SimplexVertex& interiorParams = m_pInteriorParams[ii];
      const Matrix& rotMatrix = m_pMatrix[ii];

      x = point.getXPos(); y = point.getYPos(); x0 = interiorParams(0); y0 = interiorParams(1);
      k1 = interiorParams(2); k2 = interiorParams(3); p1 = interiorParams(4); p2 = interiorParams(5);
      f = interiorParams(6); Xs = exteriorParams(0);Ys = exteriorParams(1); Zs = exteriorParams(2);
      a1 = rotMatrix(0,0); a2 = rotMatrix(0,1); a3 = rotMatrix(0,2);
      b1 = rotMatrix(1,0); b2 = rotMatrix(1,1); b3 = rotMatrix(1,2);
      c1 = rotMatrix(2,0); c2 = rotMatrix(2,1); c3 = rotMatrix(2,2);

      u = -f*(a1*(Xa-Xs)+b1*(Ya-Ys)+c1*(Za-Zs))/(a3*(Xa-Xs)+b3*(Ya-Ys)+c3*(Za-Zs)); 
      v = -f*(a2*(Xa-Xs)+b2*(Ya-Ys)+c2*(Za-Zs))/(a3*(Xa-Xs)+b3*(Ya-Ys)+c3*(Za-Zs));
      double uv2 = u*u+v*v, u2 = u*u, v2 = v*v, uv = u*v;
      deltaU = k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
      deltaV = k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);
      u = x - x0 - deltaU; v = y - y0 - deltaV;

      mA(2*ii,0) = a1*f + a3*u; mA(2*ii,1) = b1*f + b3*u; mA(2*ii,2) = c1*f + c3*u;
      mA(2*ii+1,0) = a2*f + a3*v; mA(2*ii+1,1) = b2*f + b3*v; mA(2*ii+1,2) = c2*f + c3*v;
   }

   return mA;
}
