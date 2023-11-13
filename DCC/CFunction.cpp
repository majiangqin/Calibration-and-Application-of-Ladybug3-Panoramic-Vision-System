// Implementation of CFunction classes

#include "stdafx.h"
#include "CFunction.h"

#include "Warning.h"
#include "math.h"
#include "Matrix.h"

#define EPS 1.0e-4		// Approximate square root of the machine precision

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////////
/// General Implementation of CFunction
//////////////////////////////////////////////////////////////////////////
double CFunction::Fmin(const SimplexVertex& vx) const
{
   SimplexVertex vy = (*this)(vx);
   double sum = 0.0;
   for (int ii = 0; ii < vy.getDim(); ++ii)
   {
      sum += vy(ii) * vy(ii);
   }
   return 0.5*sum;
}

Matrix CFunction::Jacobian(const SimplexVertex& vx) const
{
   const int nn = vx.getDim();
   Matrix jacobi(nn,nn);
   SimplexVertex _vx(vx);
   SimplexVertex vy = (*this)(vx);
   for (int jj = 0; jj < nn; ++jj)
   {
      double temp = _vx(jj);
      double step = EPS*abs(temp);
      if (step == 0.0) step = EPS; 
      _vx(jj) = temp + step; // Trick to reduce finite precision error
      step = _vx(jj) - temp;
      SimplexVertex vy_new = (*this)(_vx);
      _vx(jj) = temp;
      for (int ii = 0; ii < nn; ++ii)
      {
         jacobi(ii,jj) = (vy_new(ii)-vy(ii))/step;
      }
   }
   return jacobi;
}
