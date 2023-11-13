// Implementation of pic data initiate utilities

#include "stdafx.h"
#include "PicDataInitUtil.h"

#include "Warning.h"
#include "PicData.h"
#include "Matrix.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

void PicDataInitUtil::loadImageInfo(const CString& fileName, int* pWidth, int* pHeight)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(pWidth&&pHeight,"Null pointer in PicDataInitUtil::loadImageInfo");
	CFile file;
	if (file.Open(fileName, CFile::modeRead | CFile::shareDenyNone,NULL))
	{
		BITMAPFILEHEADER bmfHeader;
		if (file.Read((LPSTR)&bmfHeader,sizeof(bmfHeader)) != sizeof(bmfHeader))
		{
			DBG_WARN("Can't read file header in PicDataInitUtil::loadImageInfo");
			*pWidth = 0; *pHeight = 0;
			return;
		}
		if (bmfHeader.bfType != 0x4d42)
		{
			DBG_WARN("This is not a bmp file in PicDataInitUtil::loadImageInfo");
			*pWidth = 0; *pHeight = 0;
			return;
		}
		BITMAPINFOHEADER bmfInfo;
		if (file.Read((LPSTR)&bmfInfo,sizeof(bmfInfo)) != sizeof(bmfInfo))
		{
			DBG_WARN("Can't read file info in PicDataInitUtil::loadImageInfo");
			*pWidth = 0; *pHeight = 0;
			return;
		}
		*pWidth = bmfInfo.biWidth;
		*pHeight = bmfInfo.biHeight;
		return;
	} 
	else
	{
		*pWidth = 0; *pHeight = 0;
		return;
	}
}

void PicDataInitUtil::initPicData(const CString& fileName, const ImagePoint& center, PicData& data)
{
	CStdioFile file;
	if(file.Open(fileName,CFile::modeRead | CFile::shareDenyNone,NULL))
	{
		// The first line contains the number of points
		CString str;
		file.ReadString(str);
      if (str.Find(' ') != -1 || str.Find('.') != -1)
      {
         DBG_WARN("This is not a standard picture control file");
      }
		int pointN = _tcstoul(str, NULL, 10); // Record of point number
		data.setPointNum(pointN);
		for (int ii = 0; ii < pointN; ++ii)
		{
			// Read every line to retrieve coordinates of points
			CString str;
			file.ReadString(str);
			double array[3];
			CString strLeft(str);
			for (int jj = 0; jj < 3; ++jj)
			{
				CString strTmp = jj == 2 ? strLeft.Mid(0) : strLeft.Mid(0,strLeft.Find(','));
				array[jj] = _tcstod(strTmp, NULL);
				strLeft = strLeft.Mid(strLeft.Find(',')+1);
			}
			data.setPoint(ii, PicControlPoint((int)array[0],array[1]-center.getXPos(),array[2]-center.getYPos()));
		}
		data.sort();
	}
	else
	{
		DBG_WARN("Open file fail in PicDataInitUtil::initPicData");
		return;
	}
}

void PicDataInitUtil::initObjectData(const CString& fileName, ObjectData& data)
{
	CStdioFile file;
	if(file.Open(fileName,CFile::modeRead | CFile::shareDenyNone,NULL))
	{
		// The first line contains the number of points
		CString str;
		for (int ii = 0; ii < 5; ++ii)
		{
			file.ReadString(str); // skip the first four lines
		}
      if (str.Find(' ') != -1 || str.Find('.') != -1)
      {
         DBG_WARN("This is not a standard object control file");
      }
		int pointN = _tcstoul(str, NULL, 10); // Record of point number
		data.setPointNum(pointN);
		for (int ii = 0; ii < pointN; ++ii)
		{
			// Read every line to retrieve coordinates of points
			CString str;
			file.ReadString(str);
			double array[4];
			CString strLeft(str);
			for (int jj = 0; jj < 4; ++jj)
			{
				CString strTmp = jj == 3 ? strLeft.Mid(0) : strLeft.Mid(0,strLeft.Find(' '));
				array[jj] = _tcstod(strTmp, NULL);
				strLeft = strLeft.Mid(strLeft.Find(' ')).TrimLeft(); // Trim whitespace
			}
			data.setPoint(ii, ObjControlPoint((int)array[0],array[1],array[2],array[3]));
		}
		data.sort();
	}
	else
	{
		DBG_WARN("Open file fail in PicDataInitUtil::initObjectData");
		return;
	}
}

void PicDataInitUtil::initDistortParams(const CString& fileName, SimplexVertex& distortParams, ImagePoint& origin, 
                                        int distortParamN)
{
	CStdioFile file;
	if(file.Open(fileName,CFile::modeRead | CFile::shareDenyNone,NULL))
	{
		CString str;
		for (int ii = 0; ii < 2; ++ii)
		{
			CString strTmp;
			file.ReadString(str); // read image center
			strTmp = str.Mid(str.Find('=')+2);
			double pos = _tcstod(strTmp, NULL);
			if (ii == 0)	origin.setXPos(pos); 
			else	origin.setYPos(pos);
		}
		distortParams.setDim(distortParamN);
		for (int ii = 0; ii < distortParamN; ++ii)
		{
			CString strTmp;
			file.ReadString(str); // read distortion parameters
			strTmp = str.Mid(str.Find('=')+2);
			double param = _tcstod(strTmp, NULL);
			distortParams(ii) = param;
		}
	}
	else
	{
		DBG_WARN("Open file fail in PicDataInitUtil::initDistortParams");
		return;
	}
}

void PicDataInitUtil::loadCameraOutParams(const CString &fileName, SimplexVertex& params, Matrix& rotMatrix)
{
   params.setDim(3);
   rotMatrix.resize(3,3);

   CStdioFile file;
   if(file.Open(fileName,CFile::modeRead | CFile::shareDenyNone,NULL))
   {
      CString str;
      file.ReadString(str); // read Xs, Ys, Zs
      for (int ii = 0; ii < 3; ++ii)
      {
         CString strTmp = str.Mid(str.Find('=')+2);
         CString pos = strTmp.Left(strTmp.Find(_T("mm")));
         params(ii) = _tcstod(pos, NULL);
         str = str.Mid(str.Find(_T("mm"))+3);
      }
      for (int ii = 0; ii < 2; ++ii)
      {
         file.ReadString(str); // skip two lines
      }
      for (int ii = 0; ii < 3; ++ii) // read rotate matrix
      {
         file.ReadString(str);
         for (int jj = 0; jj < 3; ++jj)
         {
            CString strTmp = str.Left(str.Find('.')+7);
            rotMatrix(ii,jj) = _tcstod(strTmp, NULL);
            str = str.Mid(str.Find('.')+7);
         }
      }
   }
   else
   {
      DBG_WARN("Open file fail in PicDataInitUtil::loadCameraParams");
      return;
   }
}

void PicDataInitUtil::loadCameraInParams(const CString &fileName, SimplexVertex &params, int distortParamN)
{
   CStdioFile file;
   if(file.Open(fileName,CFile::modeRead | CFile::shareDenyNone,NULL))
   {
      for (int ii = 0; ii < 7; ++ii)
      {
         CString str;
         file.ReadString(str); // skip 7 lines
      }
      params.setDim(distortParamN+3);
      for (int ii = 0; ii < distortParamN+3; ++ii) // read x0, y0, c3, c5, p1, p2, f
      {
         CString str;
         file.ReadString(str);
         CString strTmp = str.Mid(str.Find('=')+2);
         double param = _tcstod(strTmp, NULL);
         params(ii) = param;
      }
   }
   else
   {
      DBG_WARN("Open file fail in PicDataInitUtil::loadCameraInParams");
      return;
   }
}