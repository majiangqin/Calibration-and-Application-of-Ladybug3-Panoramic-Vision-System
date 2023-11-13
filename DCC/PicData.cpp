// Implementation for Picture Data structure class

#include "stdafx.h"
#include "PicData.h"

#include "math.h"
#include "Warning.h"
#include "Matrix.h"
#include "MathUtil.h"
#include "CDistortFunction.h"
#include "DataAlgorithm.h"

#define RANGE_AMP 100		// The range of noise amplification
#define RANGE_ANG 360		// The range of noise direction

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////////
/// PicControlPoint: Picture Control Point Class
//////////////////////////////////////////////////////////////////////////
PicControlPoint& PicControlPoint::operator =(const PicControlPoint& point)
{
	m_index = point.getIndex();
	m_xPos = point.getXPos();
	m_yPos = point.getYPos();
	return *this;
}

bool PicControlPoint::operator <(const PicControlPoint& point)
{
	if (m_index < point.m_index)
		return true;
	else
		return false;
}

bool PicControlPoint::operator >(const PicControlPoint& point)
{
	if (m_index > point.m_index)
		return true;
	else
		return false;
}

bool PicControlPoint::operator ==(const PicControlPoint &point)
{
	if (m_index == point.m_index)
		return true;
	else
		return false;
}


//////////////////////////////////////////////////////////////////////////
/// PicData: Picture Data Class
//////////////////////////////////////////////////////////////////////////
PicData::PicData(int n)
: m_pointN(n)
{
	m_points = new PicControlPoint[n];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in PicData::PicData");
}

PicData::PicData(const PicData& data)
: m_pointN(data.m_pointN)
{
   m_points = new PicControlPoint[m_pointN];
   DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in PicData::PicData");
   for (int ii = 0; ii < m_pointN; ++ii)
   {
      m_points[ii] = data.getPoint(ii);
   }
}

PicData::~PicData()
{
	if (m_points) delete[] m_points;
	m_pointN = 0;
}

void PicData::setPoint(int i, const PicControlPoint& point)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(i >= 0 && i < m_pointN, "Point i doesn't exist in PicData::setPoint");
	m_points[i] = point;
}

PicControlPoint PicData::getPoint(int i) const
{
	DBG_WARN_AND_RETURN_UNLESS(i >= 0 && i < m_pointN, PicControlPoint(), "Point i doesn't exist in PicData::getPoint");
	return m_points[i];
}

void PicData::setPointNum(int n)
{
	if (m_pointN != n)
	{
		m_pointN = n;
		if (m_points) delete[] m_points;
		m_points = new PicControlPoint[n];
		DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in PicData::setPointNum");
	}
}

void PicData::sort()
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pointN>0,"No points in PicData::sort");
	HeapSort<PicControlPoint>::sort(m_points,m_pointN);
}

int PicData::find(int index) const
{
	PicControlPoint point(index,0.0,0.0);
	return BinarySearch<PicControlPoint>(m_points,point,m_pointN);
}


//////////////////////////////////////////////////////////////////////////
/// ObjControlPoint: Object Control Point Class
//////////////////////////////////////////////////////////////////////////
ObjControlPoint& ObjControlPoint::operator =(const ObjControlPoint& point)
{
	m_index = point.getIndex();
	m_xPos = point.getXPos();
	m_yPos = point.getYPos();
	m_zPos = point.getZPos();
	return *this;
}

bool ObjControlPoint::operator <(const ObjControlPoint &point)
{
	if (m_index < point.m_index)
		return true;
	else
		return false;
}

bool ObjControlPoint::operator >(const ObjControlPoint &point)
{
	if (m_index > point.m_index)
		return true;
	else
		return false;
}

bool ObjControlPoint::operator ==(const ObjControlPoint &point)
{
	if (m_index == point.m_index)
		return true;
	else
		return false;
}


//////////////////////////////////////////////////////////////////////////
/// ObjectData: Object Data Class
//////////////////////////////////////////////////////////////////////////
ObjectData::ObjectData(int n)
: m_pointN(n)
{
	m_points = new ObjControlPoint[n];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in ObjectData::ObjectData");
}

ObjectData::ObjectData(const ObjectData& data)
: m_pointN(data.m_pointN)
{
   m_points = new ObjControlPoint[m_pointN];
   DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in ObjectData::ObjectData");
   for (int ii = 0; ii < m_pointN; ++ii)
   {
      m_points[ii] = data.getPoint(ii);
   }
}

ObjectData::~ObjectData()
{
	if (m_points) delete[] m_points;
	m_pointN = 0;
}

void ObjectData::setPoint(int i, const ObjControlPoint& point)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(i >= 0 && i < m_pointN, "Point i doesn't exist in ObjectData::setPoint");
	m_points[i] = point;
}

ObjControlPoint ObjectData::getPoint(int i) const
{
	DBG_WARN_AND_RETURN_UNLESS(i >= 0 && i < m_pointN, ObjControlPoint(), "Point i doesn't exist in ObjectData::getPoint");
	return m_points[i];
}

void ObjectData::setPointNum(int n)
{
	if (m_pointN != n)
	{
		m_pointN = n;
		if (m_points) delete[] m_points;
		m_points = new ObjControlPoint[n];
		DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL, "Memory allocation failure in ObjectData::setPointNum");
	}
}

void ObjectData::sort()
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pointN>0,"No points in PicData::sort");
	HeapSort<ObjControlPoint>::sort(m_points,m_pointN);
}

int ObjectData::find(int index) const
{
	ObjControlPoint point(index,0.0,0.0,0.0);
	return BinarySearch<ObjControlPoint>(m_points,point,m_pointN);
}


//////////////////////////////////////////////////////////////////////////
/// ImagePoint: Picture Point Coordinate Class
//////////////////////////////////////////////////////////////////////////
ImagePoint::ImagePoint(const PicControlPoint &point)
: m_xPos(point.getXPos())
, m_yPos(point.getYPos())
{
   
}

ImagePoint& ImagePoint::operator =(const ImagePoint& point)
{
	m_xPos = point.getXPos();
	m_yPos = point.getYPos();
	return *this;
}

ImagePoint& ImagePoint::operator -= (const ImagePoint& point)
{
	m_xPos -= point.getXPos();
   m_yPos -= point.getYPos();
	return *this;
}

ImagePoint operator + (const ImagePoint& point1, const ImagePoint& point2)
{
	ImagePoint point(point1);
	point.m_xPos += point2.getXPos();
	point.m_yPos += point2.getYPos();
	return point;
}

ImagePoint operator - (const ImagePoint& point1, const ImagePoint& point2)
{
	ImagePoint point(point1);
	point.m_xPos -= point2.getXPos();
	point.m_yPos -= point2.getYPos();
	return point;
}

ImagePoint operator * (double par, const ImagePoint& point)
{
	ImagePoint point1(point);
	point1.m_xPos *= par;
	point1.m_yPos *= par;
	return point1;
}

bool ImagePoint::undistort(const CFUnDistort& func, ImagePoint* pPoint) const
{
	DBG_WARN_AND_RETURN_UNLESS(pPoint, false, "Null pPointer in ImagePoint::undistort");
	SimplexVertex vx(*this);
	bool check = Newton::solve(vx,func);
	if (check)
		return false;
	pPoint->setXPos(vx.getEntry(0));
	pPoint->setYPos(vx.getEntry(1));
	return true;
}

ImagePoint ImagePoint::distort(const CFDistort& func) const
{
	SimplexVertex newPnt = func(*this);
	DBG_WARN_AND_RETURN_UNLESS(newPnt.getDim()==2, ImagePoint(), "Image point can't have more than two coordinates in ImagePoint::distort");
	return ImagePoint(newPnt.getEntry(0), newPnt.getEntry(1));
}

ImagePoint ImagePoint::distortWithNoise(const CFDistort &func, double noiseAmple, int random) const
{
	ImagePoint distortPnt = distort(func);
	int iAmp = (int)(((double) random / (double) RAND_MAX) * noiseAmple * RANGE_AMP);
	int iAng = (int)(((double) random / (double) RAND_MAX) * RANGE_ANG);
	double ampl = (double) iAmp / (double) RANGE_AMP;
	double angl = ((double) iAng / (double) RANGE_ANG) * 2 * PI;
	double noiseX = ampl * cos(angl);
	double noiseY = ampl * sin(angl);
	double x = distortPnt.getXPos() + noiseX;
	double y = distortPnt.getYPos() + noiseY;
	distortPnt.setXPos(x); distortPnt.setYPos(y);
	return distortPnt;
}


//////////////////////////////////////////////////////////////////////////
/// ImageLine: Picture Line Class
//////////////////////////////////////////////////////////////////////////
ImageLine::ImageLine(int i, int n)
: m_index(i)
, m_pointN(n)
{
	m_points = new ImagePoint[n];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL,	"Memory allocation failure in ImageLine::ImageLine");
}

ImageLine::ImageLine(const ImageLine &line)
: m_index(line.m_index)
, m_pointN(line.m_pointN)
{
	m_points = new ImagePoint[m_pointN];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL,	"Memory allocation failure in ImageLine::ImageLine");
	for (int ii = 0; ii < m_pointN; ++ii)
	{
		m_points[ii] = line.getPoint(ii);
	}
}

ImageLine::~ImageLine()
{
	if (m_points) delete[] m_points;
	m_pointN = 0;
	m_index = 0;
}

void ImageLine::setPointNum(int n)
{
	if (m_pointN != n)
	{
		m_pointN = n;
		if (m_points) delete[] m_points;
		m_points = new ImagePoint[n];
		DBG_WARN_AND_RETURN_VOID_UNLESS(m_points != NULL,	"Memory allocation failure in ImageLine::setPointNum");
	}
}

ImagePoint ImageLine::getPoint(int i) const
{
	DBG_WARN_AND_RETURN_UNLESS(i >= 0 && i < m_pointN, ImagePoint(), "Point i doesn't exist in ImageLine::getPoint");
	return m_points[i];
}

void ImageLine::setPoint(int i, const ImagePoint& point)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(i >= 0 && i < m_pointN, "Point i doesn't exist in ImageLine::setPoint");
	m_points[i].setXPos(point.getXPos());
	m_points[i].setYPos(point.getYPos());
}