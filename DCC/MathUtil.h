// MathUtil.h: implementation of newton method

#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include "Matrix.h"

class CFunction;

typedef int Degree[3];

#pragma once

// Newton method: guarantee a global convergence
class Newton
{
public:
	// Solve equations with Newton method, which comes from p9.7 
   // <Globally Convergent Methods for Nonlinear Systems of Equations, Numerical Recipes in C, Second Edition (1992)>
	// vx: give an initial guess X for a root in n dimensions and finds the root by globally convergence
	// returns: false on a normal return and true if the routine has converged to a local minimum
	static bool solve(SimplexVertex& vx, const CFunction& func);
	
protected:
	// Find a new point along the given direction from old point where the function value has decreased "sufficiently".
	// stpmax: an input quantity that limits the length of the steps so that you don't try to
	//			  evaluate the function in regions where it is undefined or subject to overflow.
	// vg: gradient vector
	// vp: direction vector
	// func: object function
	// returns: false on a normal exit, true when xnew is too close to xold
	static bool lnsrch(const SimplexVertex& xold, const SimplexVertex& vg, const CFunction& func,
                      double fold, double stpmax, SimplexVertex& vp, SimplexVertex* xnew, double* fnew);
};

class MathUtil
{
public:
	// This method converts radian to positive degree, regardless of radian's sign
	static void RadToDeg(double radian, Degree angle);

   // Transform a radian into [0,2*PI]
   static double Radian(double radian);

   // Check whether points have coplanar, collinear or other relationships in Euclid Space. 
   // 3 means they don't have any relationship
   // 2 means they are coplanar.
   // 1 means they are collinear.
   // 0 means they are the same point.
   // -1 means geometry property can't be obtained.
   static int GetGeometryProperty(const SimplexVertex* pPoints, int pointN, double tol = 1.0e-15);
};


#endif