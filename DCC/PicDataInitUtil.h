// PicDataInitUtil.h: header file

#ifndef PIC_DATA_INIT_UTIL_H
#define PIC_DATA_INIT_UTIL_H

class PicData;
class ImagePoint;
class ObjectData;
class SimplexVertex;
class Matrix;

#pragma once

class PicDataInitUtil
{
public:
	static void loadImageInfo(const CString& fileName, int* pWidth, int* pHeight);

	static void initPicData(const CString& fileName, const ImagePoint& center, PicData& data);

	static void initObjectData(const CString& fileName, ObjectData& data);

	static void initDistortParams(const CString& fileName, SimplexVertex& distortParams, ImagePoint& origin, 
                                 int distortParamN);

   static void loadCameraOutParams(const CString& fileName, SimplexVertex& params, Matrix& rotMatrix);

   static void loadCameraInParams(const CString& fileName, SimplexVertex& params, int distortParamN);
};


#endif