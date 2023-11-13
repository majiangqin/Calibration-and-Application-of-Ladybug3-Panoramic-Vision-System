// CalibrationUtil.h: Photo Calibration class

#ifndef CALIBRATION_UTIL_H
#define CALIBRATION_UTIL_H

#pragma once

class SimplexVertex;
class ImageLine;
class ImagePoint;
class PicData;
class ObjectData;
class Matrix;
class ObjControlPoint;
class PicControlPoint;

enum ObjectMethod
{
	SquareDistance = 0, // Sum of Squared Distances
	NormalizeSquare = 1, // Normalized Sum of Squares
	ExplicitNoiseEstimate = 2, // Explicit Noise Estimation
   WithoutDistortion = 3, // No Distortion
};

enum OrientElemType
{
	InteriorOrientElem = 1, // Interior Orientation Elements
	ExteriorOrientElem = 2, // Exterior Orientation Elements
};

// Distortion parameters calibration utilities
class DistortionCalibrationUtil
{
public:
	static void calculate(const PicData& picData, int dim, ObjectMethod objectMethod, 
								 ImagePoint* pOrigin, SimplexVertex* pDistortParams);

	// Average error of undistorted algorithm is defined as the average of the absolute distances
	// between each of the undistorted points and the original sampled points.
	static double computeAverageError(const PicData& sampleData, const PicData& realData,
												 const SimplexVertex& distortParams, const ImagePoint& origin);

protected:
	// The implementation of amoeba algorithm comes from P10.4 in <Numerical Recipes in C, Second Edition (1992)> - Downhill Simplex
	static double amoeba(const ImageLine* pLines, SimplexVertex* pCoefficients, const ImagePoint& origin, 
								int lineN, int dim, SimplexVertex* pResult, ObjectMethod objectMethod, 
								double tol = 1.0e-8);

	static void initCoefficient(const SimplexVertex& center, const double* pVector, SimplexVertex* pSimplex);

	static void initImageLine(const PicData& picData, ImageLine** ppLines, int* pLineN);

private:
	// Fit a*X + b*y + c = 0 with Least-Square Method
	static bool fit(const ImageLine& line, const bool* pMarkArray, double* pa, double* pb, double* pc);

	// Sum of Squared Distances
	static double objectFunc1(const ImageLine* pLines, const ImagePoint& origin, const SimplexVertex& coefficient, int lineN);

	// Normalized Sum of Squared Distances
	static double objectFunc2(const ImageLine* pLines, const ImagePoint& origin, const SimplexVertex& coefficient, int lineN);

	// Explicit Noise Estimation
	static double objectFunc3(const ImageLine* pLines, const ImagePoint& origin, const SimplexVertex& coefficient, int lineN);

   // Without Distortion
   // Explicit Noise Estimation
   static double objectFunc4(const ImageLine* pLines, const ImagePoint& origin, const SimplexVertex& coefficient, int lineN);

	static double amotry(const ImageLine* pLines, const ImagePoint& origin,
								double (*objectFunc)(const ImageLine *, const ImagePoint &, const SimplexVertex &, int),
								int iHigh, int lineN, int dim, double fac,
								SimplexVertex* pSum, SimplexVertex* pCoefficients, double * pResult);
};

// Other parameters except distortion calibration utilities
class PhotoCalibrationUtil
{
public:
	// Resort to iterative method to solve collinear equations 
	// pExteriorParams: Xs, Ys, Zs, φ, ω, κ, f
	static void calculate(const PicData& picData, const ObjectData& objectData, 
                         ImagePoint* pOrigin, SimplexVertex* pDistortParams, 
                         SimplexVertex* pExteriorParams,
                         bool withoutDistort);

   // Resort to Cone method to compute initial exterior elements
   // See <航天器交会对接和月球车导航中视觉测量关键技术研究与应用>
   static void initExteriorParams(const PicData& picData, const ObjectData& objectData, 
                                  const ImagePoint& origin, const SimplexVertex& distortParams,
                                  double f, SimplexVertex* pExtParams);
};

// Calculate Relative Exterior Parameters between two cameras
class RelativeCalibrationUtil
{
public:
   static void calculate(const SimplexVertex& baseParams, const SimplexVertex& compareParams,
                         const Matrix& baseRotateMatrix, const Matrix& compareRotateMatrix,
                         SimplexVertex* pRelativeParams, Matrix* pRelativeRotateMatrix);
};

#endif