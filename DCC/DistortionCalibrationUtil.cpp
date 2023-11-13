// Implementation for Distortion Coefficient Calibration class

#include "stdafx.h"
#include "CalibrationUtil.h"

#include <map>
#include <memory>
#include "math.h"

#include "Warning.h"
#include "MathUtil.h"
#include "CDistortFunction.h"

#define HUGE 1.0e+30			// A huge number
#define TOLX 1.0e-7
#define TOLE 1.0e-6
#define TOLF 1.0e-7
#define ZERO 0.1

using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////////
/// Distortion Parameters Calibration Class
//////////////////////////////////////////////////////////////////////////
void DistortionCalibrationUtil::initCoefficient(const SimplexVertex& center, const double* pVector, SimplexVertex* pSimplex)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(pVector != NULL, "pVector is null in CalibrationUtil::initCoefficient");
	DBG_WARN_AND_RETURN_VOID_UNLESS(pSimplex != NULL, "pSimplex is null in CalibrationUtil::initCoefficient");

	for (int ii = 0; ii < center.getDim(); ++ii)
	{
		pSimplex[ii].setDim(center.getDim());
		for (int jj = 0; jj < center.getDim(); ++jj)
		{
			double x = ii == jj ? pVector[ii] : 0.0;
			pSimplex[ii].setEntry(jj, x);
		}
		pSimplex[ii] += center;
	}
	pSimplex[center.getDim()] = center;

   tr1::shared_ptr<double> p(new double(2));
}

void DistortionCalibrationUtil::initImageLine(const PicData& picData, ImageLine** ppLines, int* pLineN)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(pLineN != NULL, "pLineNum is null in CalibrationUtil::initImageLine");

	typedef pair<int, int> IntPair;
	map<int, int, less<int> > lineMap; // line index vs point number map, sort by ascend order
	map<int, int, less<int> > indexMap; // line index vs line order map, sort by ascend order

	// Build line index vs point number map
	for (int ii = 0; ii < picData.getPointNum(); ++ii)
	{
		PicControlPoint point = picData.getPoint(ii);
		int index = point.getLineIndex();
		map<int, int, less<int> >::iterator iterTmp = lineMap.find(index);
		if ( iterTmp != lineMap.end() )
		{
			++ iterTmp->second;
		} 
		else
		{
			lineMap.insert(IntPair(index, 1));
		}
	}
	*pLineN = (int)lineMap.size();
	// Build line index vs line order map
	int order = 0;
	for (map<int, int, less<int> >::iterator iterTmp = lineMap.begin(); iterTmp != lineMap.end(); ++iterTmp, ++order)
	{
		int index = iterTmp->first;
		indexMap.insert(IntPair(index, order));
	}

	// Initiate image lines
	*ppLines = new ImageLine[*pLineN];
	map<int, int, less<int> >::iterator iter = lineMap.begin();
	for (int ii = 0; ii < *pLineN && iter != lineMap.end(); ++ii, ++iter)
	{
		int index = iter->first;
		int pointN = iter->second;
		(*ppLines)[ii].setIndex(index);
		(*ppLines)[ii].setPointNum(pointN);
	}
	// Build image lines
	for (int ii = picData.getPointNum()-1; ii >= 0; --ii)
	{
		PicControlPoint point = picData.getPoint(ii);
		int index = point.getLineIndex();
		map<int, int, less<int> >::iterator iter1 = lineMap.find(index);
		map<int, int, less<int> >::iterator iter2 = indexMap.find(index);
		if ( iter1 != lineMap.end() && iter2 != indexMap.end() )
		{
			int order = iter2->second;
			if (!iter1->second)
			{
				DBG_WARN("Point doesn't exist in CalibrationUtil::initImageLine");
				return;
			}
			(*ppLines)[order].setPoint(iter1->second-1, ImagePoint(point.getXPos(), point.getYPos()));
			-- iter1->second;
		} 
		else
		{
			DBG_WARN("Odd line index is found in CalibrationUtil::initImageLine");
			return;
		}
	}
}

bool DistortionCalibrationUtil::fit(const ImageLine& line, const bool* pMarkArray, double* pa, double* pb, double* pc)
{
	if ( !pa || !pb || !pc || !pMarkArray )
	{
		DBG_WARN("null pointer in CalibrationUtil::fit");
		return false;
	}
	else if ( line.getPointNum() < 2 )
	{
		DBG_WARN("line has less than two points in CalibrationUtil::fit");
		return false;
	}

	int pointN = 0;
	double mX = 0.0, mY = 0.0, mXX = 0.0, mXY = 0.0, mYY = 0.0;            
	for (int ii = 0; ii < line.getPointNum(); ++ii)
	{
		if (!pMarkArray[ii])	continue;
		ImagePoint& point = line.getPoint(ii);
		double x = point.getXPos();
		double y = point.getYPos();
		mX += x;
		mY += y;
		mXX += x * x;
		mXY += x * y;
		mYY += y * y;
		++ pointN;
	}

	if ( abs(mX * mX - mXX * pointN) < ZERO && abs(mY * mY - mYY * pointN) < ZERO )
	{
		if (abs(mX*mY-mXY*pointN) < ZERO && abs(mX*mY-mYY*pointN) < ZERO && abs(mX*mYY-mY*mXY) < ZERO)
		{
			*pa = 1.0;
			*pb = -1.0;
			*pc = 1.0;
		} 
		else	
		{
			return false;
		}
	}
	else if ( abs(mX * mX - mXX * pointN) < ZERO )
	{
		double temp1 = mX*mY-mXY*pointN;
		double temp2 = mX*mY-mYY*pointN;
		if ( abs(mX*mY-mXY*pointN) < ZERO )
		{
			*pa = (mX*mY-mYY*pointN) / (mY*mY-mYY*pointN);
			*pb = 0.0;
			*pc = *pa * ((mX*mYY-mY*mXY) / (mY*mY-mYY*pointN));
		} 
		else if ( abs(mX*mY-mYY*pointN) < ZERO )
		{
			*pa = (mX*mY-mXY*pointN) / (mY*mY-mYY*pointN);
			*pb = -1.0 * *pa * ((mX*mY-mXY*pointN) / (mY*mY-mYY*pointN));
			*pc = *pa * ((mX*mYY-mY*mXY) / (mY*mY-mYY*pointN));
		}
		else
		{
			return false;
		}
	}
	else if ( abs(mY * mY - mYY * pointN) < ZERO )
	{
		if ( abs(mX*mY-mXY*pointN) < ZERO && abs(mX*mYY-mY*mXY) < ZERO )
		{
			*pa = (mX*mY-mYY*pointN) / (mX*mX-mXX*pointN);
			*pb = -1.0 * *pa;
			*pc = *pa;
		} 
		else
		{
			return false;
		}		
	}
	else
	{
		*pa = ((mX*mY-mXY*pointN) * (mX*mY-mYY*pointN)) / ((mY*mY-mYY*pointN) * (mX*mX-mXX*pointN));
		*pb = -1.0 * *pa * ((mX*mY-mXY*pointN) / (mY*mY-mYY*pointN));
		*pc = *pa * ((mX*mYY-mY*mXY) / (mY*mY-mYY*pointN));
	}
	return true;
}

double DistortionCalibrationUtil::objectFunc1(const ImageLine* pLines, const ImagePoint& origin, const SimplexVertex& coefficient, int lineN)
{
	double error = 0.0;
	int pointN = 0;
	for (int ii = 0; ii < lineN; ++ii )
	{
		const int nn = pLines[ii].getPointNum();
		ImageLine undistortLine(pLines[ii]);
		bool* pMarkArray = new bool[nn];
		int validPointN = 0;
		for (int jj = 0; jj < nn; ++jj)
		{
			ImagePoint point = undistortLine.getPoint(jj);
			CFUnDistort func(coefficient,point,origin);
			if (!point.undistort(func,&point))
			{
				pMarkArray[jj] = false;
				continue;
			}
			pMarkArray[jj] = true;
			undistortLine.setPoint(jj,point);
			++ validPointN;
		}
		if (validPointN < 2)
		{
			DBG_WARN("There are not enough valid points with current coefficients in this line in CalibrationUtil::objectFunc1");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		pointN += validPointN;
		double a = 0.0, b = 0.0, c = 0.0;
		if (!fit(undistortLine, pMarkArray, &a, &b, &c))
		{
			DBG_WARN("This line can't be fit with current coefficients in CalibrationUtil::objectFunc1");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		for (int jj = 0; jj < nn; ++jj )
		{
			if (!pMarkArray[jj])	continue;
			ImagePoint& point = undistortLine.getPoint(jj);
			double x = point.getXPos(), y = point.getYPos();
			error += (a*x+b*y+c)*(a*x+b*y+c)/(a*a+b*b);
		}
		if (pMarkArray)	delete [] pMarkArray;
	}
	return error/pointN;
}

double DistortionCalibrationUtil::objectFunc2(const ImageLine *pLines, const ImagePoint &origin, const SimplexVertex &coefficient, int lineN)
{
	double error = 0.0;
	int pointN = 0;
	for (int ii = 0; ii < lineN; ++ii )
	{
		const int nn = pLines[ii].getPointNum();
		ImageLine undistortLine(pLines[ii]);
		bool* pMarkArray = new bool[nn];
		int validPointN = 0;
		for (int jj = 0; jj < nn; ++jj)
		{
			ImagePoint point = undistortLine.getPoint(jj);
			CFUnDistort func(coefficient,point,origin);
			if (!point.undistort(func,&point))
			{
				pMarkArray[jj] = false;
				continue;
			}
			pMarkArray[jj] = true;
			undistortLine.setPoint(jj,point);
			++ validPointN;
		}
		if (validPointN < 2)
		{
			DBG_WARN("There are not enough valid points with current coefficients in this line in CalibrationUtil::objectFunc2");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		pointN += validPointN;
		double a = 0.0, b = 0.0, c = 0.0;
		if (!fit(undistortLine, pMarkArray, &a, &b, &c))
		{
			DBG_WARN("This line can't be fit with current coefficients in CalibrationUtil::objectFunc2");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		for (int jj = 0; jj < nn; ++jj )
		{
			if (!pMarkArray[jj])	continue;
			ImagePoint& point = undistortLine.getPoint(jj);
			double x = point.getXPos(), y = point.getYPos();
			double dist2 = x*x+y*y;
			error += (a*x+b*y+c)*(a*x+b*y+c)/(dist2*(a*a+b*b));
		}
		if (pMarkArray)	delete [] pMarkArray;
	}
	return error/pointN;
}

double DistortionCalibrationUtil::objectFunc3(const ImageLine * pLines, const ImagePoint &origin, const SimplexVertex &coefficient, int lineN)
{
	double error = 0.0;
	int pointN = 0;
	const double x0 = origin.getXPos(), y0 = origin.getYPos();
	for (int ii = 0; ii < lineN; ++ii )
	{
		const int nn = pLines[ii].getPointNum();
		ImageLine undistortLine(pLines[ii]);
		bool* pMarkArray = new bool[nn];
		int validPointN = 0;
		for (int jj = 0; jj < nn; ++jj)
		{
			ImagePoint point = undistortLine.getPoint(jj);
			CFUnDistort func(coefficient,point,origin);
			if (!point.undistort(func,&point))
			{
				pMarkArray[jj] = false;
				continue;
			}
			pMarkArray[jj] = true;
			undistortLine.setPoint(jj,point);
			++ validPointN;
		}
		if (validPointN < 2)
		{
			DBG_WARN("There are not enough valid points with current coefficients in this line in CalibrationUtil::objectFunc3");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		pointN += nn;
		double a = 0.0, b = 0.0, c = 0.0;
		if (!fit(undistortLine, pMarkArray, &a, &b, &c))
		{
			DBG_WARN("This line can't be fit with current coefficients in CalibrationUtil::objectFunc3");
			if (pMarkArray)	delete [] pMarkArray;
			return HUGE;
		}
		for (int jj = 0; jj < nn; ++jj )
		{
			if (!pMarkArray[jj])	continue;
			const ImagePoint& point = pLines[ii].getPoint(jj);
			double x = point.getXPos(), y = point.getYPos();
			CFindClosePoint func(coefficient, origin, point, a, b, c);
			SimplexVertex vt(1);
			if (pMarkArray[jj])
			{
				const ImagePoint& undistortPnt = undistortLine.getPoint(jj);
				double xd = undistortPnt.getXPos(), yd = undistortPnt.getYPos();
				double t = (a*xd+b*yd+c)/(a*xd-a*x0+b*yd-b*y0);
				if (abs(a)>abs(b))
				{
					vt.setEntry(0,t*y0+(1.0-t)*yd);
				} 
				else
				{
					vt.setEntry(0,t*x0+(1.0-t)*xd);
				}
			}
			else
			{
				double t = (a*x+b*y+c)/(a*x-a*x0+b*y-b*y0);
				if (abs(a)>abs(b))
				{
					vt.setEntry(0,t*y0+(1.0-t)*y);
				} 
				else
				{
					vt.setEntry(0,t*x0+(1.0-t)*x);
				}
			}
			bool check = Newton::solve(vt,func);
			double t = vt.getEntry(0);
			ImagePoint closePoint;
			if (abs(a)>abs(b))
			{
				closePoint.setYPos(t);
				closePoint.setXPos(-(c+b*t)/a);
			}
			else
			{
				closePoint.setXPos(t);
				closePoint.setYPos(-(c+a*t)/b);
			}
			CFDistort func1(coefficient,origin);
			closePoint = closePoint.distort(func1);
			double deltaX = point.getXPos() - closePoint.getXPos(), deltaY = point.getYPos() - closePoint.getYPos();
			error += (deltaX*deltaX+deltaY*deltaY)/(x*x+y*y);
		}
		if (pMarkArray)	delete [] pMarkArray;
	}
	return error/pointN;
}


namespace
{
	bool checkParamBoundary(const SimplexVertex& params, int dim)
	{
		if ( dim == 4 && params.getDim() == dim )
		{
			// Check distort parameters boundaries: C3: (-1.0e-5,1.0e-5), C5: (-1.0e-9,1.0e-9), 
			// P1: (-1.0e-5,1.0e-5), P2: (-1.0e-5,1.0e-5)
			double C3 = params.getEntry(0);
			double C5 = params.getEntry(1);
			double P1 = params.getEntry(2);
			double P2 = params.getEntry(3);
			if ( abs(C3)<1.0e-5 && abs(C5)<1.0e-9 && abs(P1)<1.0e-5 && abs(P2)<1.0e-5 )
				return true;
		}
		return false;
	};

	SimplexVertex getParamSum(const SimplexVertex* pParams, int dim)
	{
		DBG_WARN_AND_RETURN_UNLESS(pParams, SimplexVertex(), "Null pointer in getParamSum");
		SimplexVertex sum(pParams[0]);
		for (int ii = 1; ii < dim+1; ++ii)
		{
			sum += pParams[ii];
		}
		return sum;
	};
}

double DistortionCalibrationUtil::amotry(const ImageLine* pLines, const ImagePoint& origin,
													  double (*objectFunc)(const ImageLine *, const ImagePoint &, const SimplexVertex &, int),
													  int iHigh, int lineN, int dim, double fac,
													  SimplexVertex* pSum, SimplexVertex* pCoefficients, double * pResult)
{
	if (!pLines || !pCoefficients || !pResult || !pSum)
	{
		DBG_WARN("Null pointer in CalibrationUtil::amotry");
		return HUGE;
	}
	double fac1 = (1.0-fac)/dim;
	double fac2 = fac1 - fac;
	SimplexVertex param = fac1 * (*pSum) - fac2 * (pCoefficients[iHigh]);
	if (!checkParamBoundary(param,dim))
		return HUGE;
	double ftry = (*objectFunc)(pLines, origin, param, lineN);
	if (ftry < pResult[iHigh])
	{
		// Evaluate the function at the trial point. If it's better than the highest, then replace the highest.
		pResult[iHigh] = ftry;
		*pSum += (param-pCoefficients[iHigh]);
		pCoefficients[iHigh] = param;
	}
	return ftry;
}

double DistortionCalibrationUtil::amoeba(const ImageLine *pLines, SimplexVertex *pCoefficients, const ImagePoint& origin, 
													  int lineN, int dim, SimplexVertex* pResult, ObjectMethod objectMethod, double tol)
{
	if ( !pLines || !pCoefficients || !pResult )
	{
		DBG_WARN("Null pointer in CalibrationUtil::amoeba");
		return HUGE;
	}

	// Choose the type of object method
	double (*objectFunc)(const ImageLine *, const ImagePoint &, const SimplexVertex &, int);
	switch(objectMethod)
	{
	case SquareDistance:
		objectFunc = objectFunc1;
		break;
	case NormalizeSquare:
		objectFunc = objectFunc2;
	   break;
	case ExplicitNoiseEstimate:
		objectFunc = objectFunc3;
		break;
	default:
		DBG_WARN("Invalid type of object methods in CalibrationUtil::amoeba");
	   break;
	}

	int iter = 1; // record of function evaluations

	// Build an array of object function value for simplex vertex
	double* result = new double[dim+1];
	for (int ii = 0; ii < dim + 1; ++ii)
	{
		result[ii] = (*objectFunc)(pLines, origin, pCoefficients[ii], lineN);
		++ iter;
	}

	double ftol = 0.0;
	double err = 0.0;
	SimplexVertex paramSum(getParamSum(pCoefficients,dim));
	for (;;) 
	{
		// First we must determine the index of highest point, next-highest point and lowest point,
		// by looping over the points in the simplex
		int iLow = 0, iHigh = 0, ilHigh = 0;
		if (result[0] > result[1])		{ iHigh = 0; ilHigh = 1; } 
		else 		{ iHigh = 1; ilHigh = 0; }
		for (int ii = 0; ii < dim + 1; ++ii)
		{
			if (result[ii] <= result[iLow]) 
			{
				iLow = ii;
			}
			if (result[ii] > result[iHigh]) 
			{
				ilHigh = iHigh;
				iHigh = ii;
			}
			else if (result[ii] > result[ilHigh] && ii != iHigh) 
			{
				ilHigh = ii;
			}
		}

		ftol = 2.0 * abs(result[iHigh]-result[iLow])/(abs(result[iHigh])+abs(result[iLow])+TINY);
		// Compute the fractional range from highest to lowest and return if satisfactory
		if (ftol < tol)
		{
			*pResult = pCoefficients[iLow];
			err = result[iLow];
			break;
		}

		if (iter > NMAX)
		{
			DBG_WARN("NMAX exceeded in CalibrationUtil::amoeba");
			*pResult = pCoefficients[iLow];
			err = result[iLow];
			break;
		}

		// Begin a new iteration. First extrapolate by a factor -1.0 through the face of the simplex
		// across from the highest point, i.e., reflect the simplex from the highest point
		double ftry = amotry(pLines, origin, objectFunc, iHigh, lineN, dim, -1.0, &paramSum, pCoefficients, result);
		++ iter;
		if (ftry <= result[iLow])
		{
			// Give a result better than the best point, so try an additional extrapolation by a factor 2.0
			ftry = amotry(pLines, origin, objectFunc, iHigh, lineN, dim, 2.0, &paramSum, pCoefficients, result);
			++ iter;
		}
		else if (ftry >= result[ilHigh])
		{
			// The reflected point is worse than the second-highest, so look for an intermediate lower point,
			// i.e., do a one-dimension contraction
			double fsave = result[iHigh];
			ftry = amotry(pLines, origin, objectFunc, iHigh, lineN, dim, 0.5, &paramSum, pCoefficients, result);
			++ iter;
			if (ftry >= fsave)
			{
				// Can't seem to get rid of that highest point. Better contract around the lowest point
				for (int ii = 0; ii < dim + 1; ++ii)
				{
					if (ii != iLow)
					{
						paramSum = 0.5 * (pCoefficients[ii] + pCoefficients[iLow]);
						pCoefficients[ii] = paramSum;
						result[ii] = (*objectFunc)(pLines, origin, paramSum, lineN);
						++ iter;
					}
				}
				paramSum = getParamSum(pCoefficients,dim);
			}
		}
	}

	delete[] result;
	return err;
}

void DistortionCalibrationUtil::calculate(const PicData &picData, int dim, ObjectMethod objectMethod, 
														ImagePoint *pOrigin, SimplexVertex *pCoefficients)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(pOrigin&&pCoefficients,"Null pointer in DistortionCalibrationUtil::calculate");

	double* pParamArray = new double[dim];
	DBG_WARN_AND_RETURN_VOID_UNLESS(pParamArray != NULL, "Memory allocation failure in DistortionCalibrationUtil::calculate");
	if (dim == 4)
	{
		// {0.99e-5, 0.99e-9, 0.99e-5, 0.99e-5};
		for (int ii = 0; ii < dim; ++ii)
		{
			if (ii != 1)	pParamArray[ii] = 0.99e-5;
			else	pParamArray[ii] = 0.99e-9;
		}
	}
	else
	{
		DBG_WARN("Non-four distortion parameter mode has not been supported");
		return;
	}

	SimplexVertex center(dim);
	SimplexVertex* pSimplex = new SimplexVertex[dim+1];
	DBG_WARN_AND_RETURN_VOID_UNLESS(pSimplex != NULL, "Memory allocation failure in DistortionCalibrationUtil::calculate");

	double lowErr = HUGE;
	SimplexVertex distortParams(dim);
	pOrigin->setXPos(0.0); pOrigin->setYPos(0.0);
	const double pixal = 1; // length of one pixal
	ImageLine* pLines;
	int lineN; // image line amount
	initImageLine(picData, &pLines, &lineN);
	for (int ii = 0; ii < 3; ++ii)
	{
		double interval = ii == 0 ? 10*pixal : (ii == 1 ? 4*pixal : 1*pixal); // coarse-to-fine search
		ImagePoint oriTemp(*pOrigin);
		for (int jj = 0; jj < 25; ++jj)
		{
			// 5*5 Grid
			int row = jj/5 - 2;
			int col = jj%5 - 2;
			ImagePoint point(pOrigin->getXPos()+row*interval, pOrigin->getYPos()+col*interval);
			initCoefficient(center, pParamArray, pSimplex);
			double err = amoeba(pLines, pSimplex, point, lineN, dim, &distortParams, objectMethod, TOLF);
			if ( err < lowErr )
			{
				lowErr = err;
				oriTemp = point;
				*pCoefficients = distortParams;
			}
		}
		*pOrigin = oriTemp;
	}

	if (pParamArray)	delete[] pParamArray;
	if (pLines)	delete[] pLines;
	if (pSimplex)	delete[] pSimplex;
}

double DistortionCalibrationUtil::computeAverageError(const PicData& sampleData, const PicData& realData, 
																		const SimplexVertex &distortParams, const ImagePoint &origin)
{
	DBG_WARN_AND_RETURN_UNLESS(sampleData.getPointNum()==realData.getPointNum(), HUGE, "The sample data can't match the real data in CalibrationUtil::computeAverageError");
	const int nn = sampleData.getPointNum();
	double error = 0.0;
	for (int ii = 0; ii < nn; ++ii)
	{
		ImagePoint samplePoint(sampleData.getPoint(ii).getXPos(),sampleData.getPoint(ii).getYPos());
		ImagePoint realPoint(realData.getPoint(ii).getXPos(),realData.getPoint(ii).getYPos());
		CFUnDistort func(distortParams,realPoint,origin);
		ImagePoint undistortPoint;
		if (!realPoint.undistort(func,&undistortPoint))
		{
			DBG_WARN("Undistorting point fails in CalibrationUtil::computeAverageError");
		}
		double dist2 = (samplePoint.getXPos()-undistortPoint.getXPos())*(samplePoint.getXPos()-undistortPoint.getXPos())
						  +(samplePoint.getYPos()-undistortPoint.getYPos())*(samplePoint.getYPos()-undistortPoint.getYPos());
		error += sqrt(dist2);
	}
	return error/nn;
}

