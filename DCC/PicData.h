// PicData.h: data strcuture class for picture

#ifndef PIC_DATA_H
#define PIC_DATA_H

class SimplexVertex;
class CFDistort;
class CFUnDistort;

#pragma once

class PicControlPoint
{
public:
	PicControlPoint() : m_index(0), m_xPos(0.0), m_yPos(0.0) {};
	PicControlPoint(int i, double x, double y) : m_index(i), m_xPos(x), m_yPos(y) {};
	~PicControlPoint() {};

	int getIndex() const { return m_index; };
	void setIndex(int i) { m_index = i; };

	int getLineIndex() const { return m_index/100; };

	double getXPos() const { return m_xPos; };
	void setXPos(double x) { m_xPos = x; };
	double getYPos() const { return m_yPos; };
	void setYPos(double y) { m_yPos = y; };

	PicControlPoint& operator = (const PicControlPoint& point);
	bool operator < (const PicControlPoint& point);
	bool operator > (const PicControlPoint& point);
	bool operator == (const PicControlPoint& point);

private:
	int m_index;
	double m_xPos;
	double m_yPos;
};

class PicData
{
public:
	PicData() : m_pointN(0), m_points(NULL) {};
	PicData(int n);
   PicData(const PicData& data);
	~PicData();

	void setPoint(int i, const PicControlPoint& point);
	PicControlPoint getPoint(int i) const;

	int getPointNum() const { return m_pointN; };
	void setPointNum(int n);

	void sort();

	// pntIndex: PicControlPoint's index
	// returns: -1 if nothing is found, otherwise the point's location in the array
	int find(int pntIndex) const;

private:
	int m_pointN;
	PicControlPoint* m_points;
};

class ObjControlPoint
{
public:
	ObjControlPoint() : m_index(0), m_xPos(0.0), m_yPos(0.0), m_zPos(0.0) {};
	ObjControlPoint(int i, double x, double y, double z) : m_index(i), m_xPos(x), m_yPos(y), m_zPos(z) {};
	~ObjControlPoint() {};

	int getIndex() const { return m_index; };
	void setIndex(int i) { m_index = i; };

	int getLineIndex() const { return m_index/100; };

	double getXPos() const { return m_xPos; };
	void setXPos(double x) { m_xPos = x; };
	double getYPos() const { return m_yPos; };
	void setYPos(double y) { m_yPos = y; };
	double getZPos() const { return m_zPos; };
	void setZPos(double z) { m_zPos = z; };

	ObjControlPoint& operator = (const ObjControlPoint& point);
	bool operator < (const ObjControlPoint& point);
	bool operator > (const ObjControlPoint& point);
	bool operator == (const ObjControlPoint& point);

private:
	int m_index;
	double m_xPos;
	double m_yPos;
	double m_zPos;
};

class ObjectData
{
public:
	ObjectData() : m_pointN(0), m_points(NULL) {};
	ObjectData(int n);
   ObjectData(const ObjectData& data);
	~ObjectData();

	void setPoint(int i, const ObjControlPoint& point);
	ObjControlPoint getPoint(int i) const;

	int getPointNum() const { return m_pointN; };
	void setPointNum(int n);

	void sort();

	// pntIndex: ObjControlPoint's index
	// returns: -1 if nothing is found, otherwise the point's location in the array
	int find(int pntIndex) const;

private:
	int m_pointN;
	ObjControlPoint* m_points;
};

class ImagePoint
{
public:
	ImagePoint() : m_xPos(0.0), m_yPos(0.0) {};
	ImagePoint(double x, double y) : m_xPos(x), m_yPos(y) {};
   ImagePoint(const PicControlPoint& point);
	~ImagePoint() {};

	double getXPos() const { return m_xPos; };
	void setXPos(double x) { m_xPos = x; };
	double getYPos() const { return m_yPos; };
	void setYPos(double y) { m_yPos = y; };

	ImagePoint& operator = (const ImagePoint& point);
	ImagePoint& operator -= (const ImagePoint& point);
	friend ImagePoint operator + (const ImagePoint& point1, const ImagePoint& point2);
	friend ImagePoint operator - (const ImagePoint& point1, const ImagePoint& point2);
	friend ImagePoint operator * (double par, const ImagePoint& point);

   bool undistort(const CFUnDistort& func, ImagePoint* pPoint) const;
	ImagePoint distort(const CFDistort& func) const;
	ImagePoint distortWithNoise(const CFDistort& func, double noiseAmple, int random) const;

private:
	double m_xPos;
	double m_yPos;
};

class ImageLine
{
public:
	ImageLine() : m_index(0), m_pointN(0), m_points(NULL) {};
	ImageLine(int i, int n);
	ImageLine(const ImageLine& line);
	~ImageLine();

	int getIndex() const { return m_index; };
	void setIndex(int i) { m_index = i; };

	int getPointNum() const { return m_pointN; };
	void setPointNum(int n);

	void setPoint(int i, const ImagePoint& point);
	ImagePoint getPoint(int i) const;

private:
	int m_index;
	int m_pointN;
	ImagePoint* m_points;
};

#endif