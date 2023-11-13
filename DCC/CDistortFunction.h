#ifndef DISTORT_FUNCTION_H
#define DISTORT_FUNCTION_H

#include "CFunction.h"
#include "PicData.h"

#pragma once

class CFUnDistort : public CFunction
{
public:
   CFUnDistort(const SimplexVertex& params, const SimplexVertex& mapX, const ImagePoint& origin);
   virtual ~CFUnDistort() {};

   // vx: absolute coordinate vector of image point
   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

private:
   SimplexVertex m_params; // distort coefficients
   SimplexVertex m_mapX; // absolute coordinate vector of map image point
   ImagePoint m_origin; // original point
};

class CFDistort : public CFunction
{
public:
   CFDistort(const SimplexVertex& params, const ImagePoint& origin);
   virtual ~CFDistort() {};

   // vx: absolute coordinate vector of image point
   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

private:
   SimplexVertex m_params; // distort coefficients
   ImagePoint m_origin; // original point
};

class CFindClosePoint : public CFunction
{
public:
   CFindClosePoint(const SimplexVertex& params, const ImagePoint& origin, const ImagePoint& point, double a, double b, double c);
   virtual ~CFindClosePoint() {};

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

private:
   SimplexVertex m_params; // distort coefficients
   ImagePoint m_origin; // original point
   ImagePoint m_point; // target point
   double m_a;
   double m_b;
   double m_c;
};

#endif