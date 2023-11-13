#include "stdafx.h"
#include "CDistortFunction.h"

#include "Warning.h"
#include "math.h"

#define EPS 1.0e-4		// Approximate square root of the machine precision

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////////
/// Implementation of CFUndistort
//////////////////////////////////////////////////////////////////////////
CFUnDistort::CFUnDistort(const SimplexVertex& params, const SimplexVertex& mapX, const ImagePoint& origin)
: m_params(params)
, m_mapX(mapX)
, m_origin(origin)
{
}

SimplexVertex CFUnDistort::operator()(const SimplexVertex& vx) const
{
   DBG_WARN_AND_RETURN_UNLESS(vx.getDim() == 2 && m_mapX.getDim() == 2, SimplexVertex(), "This function can't be applied to non-two variables");
   DBG_WARN_AND_RETURN_UNLESS(m_params.getDim() == 4, SimplexVertex(), "This function can't be applied to non-four parameters");
   SimplexVertex vy(2);
   double k1 = m_params.getEntry(0);
   double k2 = m_params.getEntry(1);
   double p1 = m_params.getEntry(2);
   double p2 = m_params.getEntry(3);
   double u = vx.getEntry(0) - m_origin.getXPos();
   double v = vx.getEntry(1) - m_origin.getYPos();
   double uv2 = u*u+v*v; double u2 = u*u; double v2 = v*v; double uv = u*v;
   double fu = vx.getEntry(0) + k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv - m_mapX.getEntry(0);
   double fv = vx.getEntry(1) + k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2) - m_mapX.getEntry(1);
   vy.setEntry(0,fu); vy.setEntry(1,fv);
   return vy;
}

//////////////////////////////////////////////////////////////////////////
/// Implementation of CFDistort
//////////////////////////////////////////////////////////////////////////
CFDistort::CFDistort(const SimplexVertex& params, const ImagePoint& origin)
: m_params(params)
, m_origin(origin)
{
}

SimplexVertex CFDistort::operator ()(const SimplexVertex &vx) const
{
   DBG_WARN_AND_RETURN_UNLESS(vx.getDim() == 2, SimplexVertex(), "This function can't be applied to non-two variables");
   DBG_WARN_AND_RETURN_UNLESS(m_params.getDim() == 4, SimplexVertex(), "This function can't be applied to non-four parameters");
   SimplexVertex vy(2);
   double k1 = m_params.getEntry(0);
   double k2 = m_params.getEntry(1);
   double p1 = m_params.getEntry(2);
   double p2 = m_params.getEntry(3);
   double u = vx.getEntry(0) - m_origin.getXPos();
   double v = vx.getEntry(1) - m_origin.getYPos();
   double uv2 = u*u+v*v; double u2 = u*u; double v2 = v*v; double uv = u*v;
   double fu = vx.getEntry(0) + k1*u*uv2 + k2*u*uv2*uv2 + p1*(3*u2+v2) + 2*p2*uv;
   double fv = vx.getEntry(1) + k1*v*uv2 + k2*v*uv2*uv2 + 2*p1*uv + p2*(u2+3*v2);
   vy.setEntry(0,fu); vy.setEntry(1,fv);
   return vy;
}

//////////////////////////////////////////////////////////////////////////
/// Implementation of CFindClosePoint
//////////////////////////////////////////////////////////////////////////
CFindClosePoint::CFindClosePoint(const SimplexVertex& params, 
                                 const ImagePoint& origin, 
                                 const ImagePoint& point, 
                                 double a, double b, double c)
                                 : m_params(params)
                                 , m_origin(origin)
                                 , m_point(point)
                                 , m_a(a)
                                 , m_b(b)
                                 , m_c(c)
{
}

SimplexVertex CFindClosePoint::operator ()(const SimplexVertex &vx) const
{
   DBG_WARN_AND_RETURN_UNLESS(vx.getDim() == 1, SimplexVertex(), "This function can't be applied to non-one variables");
   DBG_WARN_AND_RETURN_UNLESS(m_params.getDim() == 4, SimplexVertex(), "This function can't be applied to non-four parameters");
   SimplexVertex vy(1);
   double x = 0.0, y = 0.0;
   if (abs(m_a)>abs(m_b))
   {
      y = vx.getEntry(0);
      x = -(m_b*y+m_c)/m_a;
   }
   else
   {
      x = vx.getEntry(0);
      y = -(m_a*x+m_c)/m_b;
   }
   CFDistort func(m_params, m_origin);
   ImagePoint point(x,y);
   ImagePoint closePoint = point.distort(func);
   double val = (m_point.getXPos()-closePoint.getXPos())*(m_point.getXPos()-closePoint.getXPos())
      +(m_point.getYPos()-closePoint.getYPos())*(m_point.getYPos()-closePoint.getYPos());
   vy.setEntry(0,sqrt(val));
   return vy;
}