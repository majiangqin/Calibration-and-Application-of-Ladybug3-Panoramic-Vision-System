// CFunction.h: definition of Function class

#ifndef CFUNCTION_H
#define CFUNCTION_H

#include "Matrix.h"

#pragma once

class CFunction
{
public:
	virtual ~CFunction() {};

	virtual SimplexVertex operator()(const SimplexVertex&) const = 0 {};

   // Computes forward-difference approximation to Jacobian
   virtual Matrix Jacobian(const SimplexVertex&) const;

   // Returns f = 0.5*F*F at x
   virtual double Fmin(const SimplexVertex& vx) const;
};

#endif