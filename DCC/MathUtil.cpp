// Implementation of newton method

#include "stdafx.h"
#include "MathUtil.h"

#include "math.h"
#include "Warning.h"
#include "CFunction.h"

#define MAXITS 200		// The maximum number of iterations
#define ALF 1.0e-4		// Ensure sufficient decrease in function value
#define TOLF 1.0e-4		// The convergence criterion on function values
#define TOLMIN 1.0e-6	// The criterion for deciding whether spurious convergence to a minimum of fmin has occurred
#define TOLX 1.0e-7		// The convergence criterion on deltaX
#define STPMX 100.0		// The scaled maximum step length allowed in line searches

#define FMAX(a,b) ((a) >= (b) ? (a) : (b))

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

bool Newton::solve(SimplexVertex& vx, const CFunction& func)
{
	const int nn = vx.getDim();
   SimplexVertex fvec = func(vx);
	double f = func.Fmin(vx);
   const int mm = fvec.getDim();

	double stpmax, fold;
	SimplexVertex xold(nn);
	SimplexVertex vg(nn);
	SimplexVertex dx(nn);
   Matrix vy(mm,1);
	Matrix fjac(mm, nn);
	bool check = false;

	double test = 0.0;
	for (int ii = 0; ii < mm; ++ii)
	{
		// Test for initial guess being a root. Use more stringent test than simply TOLF
		if (abs(fvec(ii)) > test)
		{
			test = abs(fvec(ii));
		}
	}
	if (test < 0.01*TOLF)
	{
		return false;
	}

	// Calculate stpmax for line searches
	double sum = 0.0;
	for (int ii = 0; ii < nn; ++ii)
	{
		sum += vx(ii) * vx(ii);
	}
	stpmax = STPMX * FMAX(sqrt(sum),(double)nn);

	// Start of iteration loop
	for (int its = 0; its < MAXITS; ++its)
	{
		// If analytic Jacobian is available, you can replace the routine fjac below with your own routine
		fjac = func.Jacobian(vx);
		for (int ii = 0; ii < nn; ++ii)
		{
			double sum = 0.0;
			for (int jj = 0; jj < mm; ++jj)
			{
				sum += fjac(jj,ii) * fvec(jj);
			}
			vg(ii) = sum; // compute deltaF for the line search
		}
      xold = vx; // Store X
		fold = f; // Store f
      vy = -1.0*fvec;
		dx = LinearEquations::solve(fjac,vy); // Solve linear equations
		check = lnsrch(xold,vg,func,fold,stpmax,dx,&vx,&f); // Returns new X and f
		fvec = func(vx);
		test = 0.0; // Test for convergence on function values
		for (int ii = 0; ii < mm; ++ii)
		{
			if (abs(fvec(ii)) > test)
			{
				test = abs(fvec(ii));
			}
		}
		if (test < TOLF)
		{
			return false;
		}
		if (check)
		{
			// Check for gradient of f zero, i.e., spurious convergence
			test = 0.0;
			double den = FMAX(f,0.5*nn);
			for (int ii = 0; ii < nn; ++ii)
			{
				double temp = abs(vg(ii))*FMAX(abs(vx(ii)),1.0)/den;
				if (temp > test)	test = temp;
			}
			return test < TOLMIN ? true : false;
		}
		test = 0.0; // Test for convergence on deltaX
		for (int ii = 0; ii < nn; ++ii)
		{
			double temp = abs(vx(ii)-xold(ii)) / FMAX(abs(vx(ii)),1.0);
			if (temp > test)	test = temp;
		}
		if (test < TOLX) return false;
	}
	//DBG_WARN("MAXITS exceeded in Newton::solve");
	return true;
}

bool Newton::lnsrch(const SimplexVertex& xold, const SimplexVertex& vg, const CFunction& func,
                    double fold, double stpmax, SimplexVertex& dx, SimplexVertex* xnew, double* fnew)
{
	DBG_WARN_AND_RETURN_UNLESS(xnew&&fnew, true, "Null pointer is input in Newton::lnsrch");
	
	const int nn = xold.getDim();
	double sum = 0.0;
	for (int ii = 0; ii < nn; ++ii)
	{
		sum += dx(ii)*dx(ii);
	}
	sum = sqrt(sum);
	if (sum > stpmax)
	{
		// Scale if attempted step is too big
		for (int ii = 0; ii < nn; ++ii)
		{
			dx(ii) *= stpmax/sum;
		}
	}

	double slope = 0.0;
	for (int ii = 0; ii < nn; ++ii)
	{
		slope += vg(ii)*dx(ii);
	}
	//if (slope >= 0.0)
	//{
	//	DBG_WARN("Roundoff problem in Newton::lnsrch");
	//}

	double test = 0.0; // compute alamin
	for (int ii = 0; ii < nn; ++ii)
	{
		double temp = abs(dx(ii)) / FMAX(abs(xold(ii)), 1.0);
		if (temp > test)	test = temp;
	}
	double alamin = TOLX/test;
	double alam = 1.0; // Always try full Newton step first
	double alam2, f2, tmplam;
	for (;;)
	{
		for (int ii = 0; ii < nn; ++ii)
		{
			double temp = xold(ii) + alam * dx(ii);
			xnew->setEntry(ii,temp);
		}
		*fnew = func.Fmin(*xnew);
		if (alam < alamin)
		{
			for (int ii = 0; ii < nn; ++ii)
			{
				// Convergence on deltaX. For zero finding, the calling program should verify the convergence.
				xnew->setEntry(ii,xold(ii));
				return true;
			}
		}
		else if (*fnew < fold+ALF*alam*slope)
		{
			// Sufficient function decrease
			return false;
		}
		else
		{
			// Backtrack
			if (alam == 1.0)
			{
				tmplam = -1.0 * slope / (2.0 * (*fnew-fold-slope));
			} 
			else
			{
				double rhs1 = *fnew - fold - alam*slope;
				double rhs2 = f2 - fold - alam2*slope;
				double a = (rhs1/(alam*alam)-rhs2/(alam2*alam2)) / (alam-alam2);
				double b = (-alam2*rhs1/(alam*alam)+alam*rhs2/(alam2*alam2)) / (alam-alam2);
				if (a == 0.0)	
				{
					tmplam = -slope/(2.0*b);
				}
				else
				{
					double disc = b*b - 3.0*a*slope;
					if (disc < 0.0)	tmplam = 0.5*alam;
					else if (b <= 0.0)	tmplam = (-b+sqrt(disc))/(3.0*a);
					else	tmplam = -slope/(b+sqrt(disc));
				}
				if (tmplam > 0.5*alam)
				{
					tmplam = 0.5*alam; // alam <= 0.5*alam1
				}
			}
		}
		alam2 = alam;
		f2 = *fnew;
		alam = FMAX(tmplam, 0.1*alam); // alam >= 0.1*alam1, (alam1 is the original alam)
	}
}


//////////////////////////////////////////////////////////////////////////
/// MathUtil
//////////////////////////////////////////////////////////////////////////
void MathUtil::RadToDeg(double radian, Degree angle)
{
   double absRad = fmod(abs(radian), 2*PI);
	const double OneDeg = PI/180.0;
	const double OneMin = OneDeg/60.0;
	const double OneSec = OneMin/60.0;
	angle[0] = int(absRad/OneDeg);
	angle[1] = int((absRad - angle[0]*OneDeg)/OneMin);
	angle[2] = int((absRad - angle[0]*OneDeg - angle[1]*OneMin)/OneSec);
}

double MathUtil::Radian(double radian)
{
   return fmod(radian, 2*PI);
}

int MathUtil::GetGeometryProperty(const SimplexVertex* pPoints, int pointN, double tol)
{
   DBG_WARN_AND_RETURN_UNLESS(pPoints,-1,"Null pointer in MathUtil::GetGeometryProperty");

   const int dim = pPoints[0].getDim();
   for (int ii = 1; ii < pointN; ++ii)
   {
      if (pPoints[ii].getDim() != dim)
      {
         DBG_WARN("Not all points have the same dimension in MathUtil::GetGeometryProperty");
         return -1;
      }
   }

   Matrix spaceMatrix(dim,pointN);
   for (int jj = 0; jj < pointN; ++jj)
   {
      for (int ii = 0; ii < dim; ++ii)
      {
         spaceMatrix(ii,jj) = pPoints[jj].getEntry(ii);
      }
   }

   int rankOri = min(dim,pointN);
   if (rankOri < 3)
      return rankOri;

   // QR decomposition
   for (int jj = 0; jj < rankOri; ++jj)
   {
      // Move pivoting element if zero
      if (abs(spaceMatrix(jj,jj)) < tol)
      {
         int sub = jj;
         for (int ii = jj + 1; ii < dim; ++ii)
         {
            if (abs(spaceMatrix(ii,jj)) > tol)
            {
               sub = ii;
               break;
            }
         }
         if (sub == jj)
            continue; // No nonzero elements found!
         for (int kk = 0; kk < pointN; ++kk)
         {
            double val = spaceMatrix(jj,kk);
            spaceMatrix(jj,kk) = spaceMatrix(sub,kk);
            spaceMatrix(sub,kk) = val;
         }
      }

      for (int ii = jj+1; ii < dim; ++ii)
      {
         double xi = spaceMatrix(jj,jj);
         double xj = spaceMatrix(ii,jj);
         if (xj == 0.0)
            continue;
         double domain = sqrt(xi*xi+xj*xj);
         double ci = xi/domain, si = xj/domain;
         for (int kk = 0; kk < pointN; ++kk)
         {
            double xk1 = spaceMatrix(jj,kk);
            double xk2 = spaceMatrix(ii,kk);
            if ( xk1 == 0.0 && xk2 == 0.0 )
               continue;
            spaceMatrix(jj,kk) = ci*xk1+si*xk2;
            spaceMatrix(ii,kk) = -si*xk1+ci*xk2;
         }
      }
   }

   int rank = rankOri;
   for (int ii = 0; ii < rankOri; ++ii)
   {
      bool isZeroRow = true;
      for (int jj = 0; jj < pointN; ++jj)
      {
         if (abs(spaceMatrix(ii,jj)) >= tol)
         {
            isZeroRow = false;
            break;
         }
      }
      if (isZeroRow) --rank;
   }

   if (rank >= 3)
      return 3;
   else
      return rank;
}