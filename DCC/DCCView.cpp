// DCCView.cpp : implementation of the CDCCView class
//

#include "stdafx.h"
#include "DCC.h"

#include "DCCDoc.h"
#include "DCCView.h"

#include <stdlib.h>
#include <time.h>

#include "CalibrationUtil.h"
#include "Warning.h"
#include "CFunction.h"
#include "CDistortFunction.h"
#include "PicDataInitUtil.h"
#include "MathUtil.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CDCCView

IMPLEMENT_DYNCREATE(CDCCView, CFormView)

BEGIN_MESSAGE_MAP(CDCCView, CFormView)
	ON_BN_CLICKED(IDC_BLOAD, &CDCCView::OnBnClickedBloadimage)
	ON_BN_CLICKED(IDC_BTEST, &CDCCView::OnBnClickedBcalibration)
	ON_CBN_SELCHANGE(IDC_OBJ_FUNC_COMBO, &CDCCView::OnCbnSelchangeObjFuncCombo)
	ON_BN_CLICKED(IDC_BDISTORT, &CDCCView::OnBnClickedBdistort)
	ON_BN_CLICKED(IDC_BLOADSAMPLE, &CDCCView::OnBnClickedBloadsample)
	ON_BN_CLICKED(IDC_BSAVE, &CDCCView::OnBnClickedBsave)
	ON_BN_CLICKED(IDC_CHECKDISTORT, &CDCCView::OnBnClickedCheckdistort)
   ON_BN_CLICKED(IDC_BCREATESAMPLE, &CDCCView::OnBnClickedBcreatesample)
END_MESSAGE_MAP()

// CDCCView construction/destruction

CDCCView::CDCCView()
	: CFormView(CDCCView::IDD)
	, m_realPicData()
	, m_samplePicData()
	, m_objectData()
	, m_imageFolderPath("")
	, m_sampleFilePath("")
	, m_distortParams()
	, m_exteriorParams()
	, m_origin(0.0,0.0)
	, m_avgErr(0.0)
	, m_distortParamN(4)
	, m_nPicWidth(0)
	, m_nPicHeight(0)
	, m_funcType(ExplicitNoiseEstimate)
	, m_orientElemType(ExteriorOrientElem)
{
	EnableActiveAccessibility();
	// TODO: add construction code here
}

CDCCView::~CDCCView()
{

}

void CDCCView::DoDataExchange(CDataExchange* pDX)
{
   CFormView::DoDataExchange(pDX);
   DDX_Control(pDX, IDC_OBJ_FUNC_COMBO, m_objFuncComboBox);
   DDX_Control(pDX, IDC_CHECKDISTORT, m_bCheckDistort);
   DDX_Control(pDX, IDC_BLOADSAMPLE, m_bLoadSample);
   DDX_Control(pDX, IDC_BDISTORT, m_bDistort);
   DDX_Control(pDX, IDC_BTEST, m_bCalibration);
   DDX_Control(pDX, IDC_BSAVE, m_bSave);
   DDX_Control(pDX, IDC_BCREATESAMPLE, m_bCreateSample);
}

BOOL CDCCView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CFormView::PreCreateWindow(cs);
}

void CDCCView::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();
	GetParentFrame()->RecalcLayout();
	ResizeParentToFit();

	// Build Object-Method ComboBox
	m_objFuncComboBox.ResetContent(); // Clear
	m_objFuncComboBox.InsertString(0, _T("Sum of Squared Distances"));
	m_objFuncComboBox.InsertString(1, _T("Normalized Sum of Squares"));
	m_objFuncComboBox.InsertString(2, _T("Explicit Noise Estimation"));
   m_objFuncComboBox.InsertString(3, _T("None"));
	m_objFuncComboBox.SetCurSel(2);

	// Initiate Buttons' State
	m_bCheckDistort.SetCheck(0);
	m_bLoadSample.EnableWindow(FALSE);
	m_bDistort.EnableWindow(FALSE);
	m_bCalibration.EnableWindow(FALSE);
	m_bSave.EnableWindow(FALSE);
   m_bCreateSample.EnableWindow(FALSE);
}


// CDCCView diagnostics

#ifdef _DEBUG
void CDCCView::AssertValid() const
{
	CFormView::AssertValid();
}

void CDCCView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}

CDCCDoc* CDCCView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CDCCDoc)));
	return (CDCCDoc*)m_pDocument;
}
#endif //_DEBUG


// CDCCView message handlers
void CDCCView::OnBnClickedBloadimage()
{
	// TODO: Add your control notification handler code here
	CFileDialog fileDlg(TRUE, _T("*.bmp"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Image File(*.bmp)|*.bmp|Image Data File(*.ctr)|*.ctr||"), this);
	fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
	if ( fileDlg.DoModal() == IDOK )
	{
		if (fileDlg.GetFileExt() == _T("ctr"))
		{
			// Purely image control points file
			CString filePath = fileDlg.GetPathName();
			m_imageFolderPath = filePath.Left(filePath.Find(fileDlg.GetFileName()));
         CString bmpFileName = filePath.Left(filePath.Find('.')+1) + _T("bmp");
			PicDataInitUtil::loadImageInfo(bmpFileName,&m_nPicWidth,&m_nPicHeight);
			PicDataInitUtil::initPicData(filePath,ImagePoint(m_nPicWidth/2.0,m_nPicHeight/2.0),m_realPicData);
			m_bCalibration.EnableWindow(TRUE);
			m_orientElemType = InteriorOrientElem;
		}
		else
		{
			CString filePath = fileDlg.GetPathName();
			m_imageFolderPath = filePath.Left(filePath.Find(fileDlg.GetFileName()));
			CString objectFileName = m_imageFolderPath + _T("控制点坐标.txt");
			CString picFileName = filePath.Left(filePath.Find('.')+1) + _T("ctr");
			CString ioeFileName = m_imageFolderPath + _T("畸变参数.txt");      // Interior orientation elements file name
			PicDataInitUtil::loadImageInfo(filePath,&m_nPicWidth,&m_nPicHeight);
			PicDataInitUtil::initPicData(picFileName,ImagePoint(m_nPicWidth/2.0,m_nPicHeight/2.0),m_realPicData);
			PicDataInitUtil::initObjectData(objectFileName,m_objectData);
			PicDataInitUtil::initDistortParams(ioeFileName,m_distortParams,m_origin,m_distortParamN);
			m_bCalibration.EnableWindow(TRUE);
			m_orientElemType = ExteriorOrientElem;
		}
	}
}

void CDCCView::OnBnClickedBcalibration()
{
	// TODO: Add your control notification handler code here
	if (m_orientElemType == InteriorOrientElem)
	{
      // Calibration of Interior Orientation Elements
      DistortionCalibrationUtil::calculate(m_realPicData,m_distortParamN,m_funcType,&m_origin,&m_distortParams);

      if (m_bCheckDistort.GetCheck())
      {
         m_avgErr = DistortionCalibrationUtil::computeAverageError(m_samplePicData, m_realPicData, m_distortParams, m_origin);
      }
	}
	else if (m_orientElemType == ExteriorOrientElem)
	{
		// Calibration of Exterior Orientation Elements
      bool withoutDistort = m_funcType == WithoutDistortion;
		PhotoCalibrationUtil::initExteriorParams(m_realPicData,m_objectData,m_origin,m_distortParams,375,&m_exteriorParams);
		PhotoCalibrationUtil::calculate(m_realPicData, m_objectData, &m_origin, &m_distortParams, &m_exteriorParams,
                                      withoutDistort);
	}
	else
	{
		DBG_WARN("Unknown OrientElemType is found in CDCCView::OnBnClickedBcalibration");
		return;
	}

	// Update Save Button
	m_bSave.EnableWindow(TRUE);
}

void CDCCView::OnCbnSelchangeObjFuncCombo()
{
	// TODO: Add your control notification handler code here
	if (m_objFuncComboBox.GetCurSel() != -1)
	{
		m_funcType = (ObjectMethod)m_objFuncComboBox.GetCurSel();
	}
}

void CDCCView::OnBnClickedBdistort()
{
	// TODO: Add your control notification handler code here
	CStdioFile file;
	CString wfileName = m_sampleFilePath.Mid(0, m_sampleFilePath.Find('.'));
	wfileName += "_distort_with_noise.ctr";
	if(file.Open(wfileName,CFile::modeCreate | CFile::modeWrite,NULL))
	{
		CString str;
		str.Format(_T("%d\n"), m_samplePicData.getPointNum());
		file.WriteString(str);
		// Seed the random-number generator with current time so that
		// the numbers will be different every time we run
		srand( (unsigned)time( NULL ) );
		for (int ii = 0; ii < m_samplePicData.getPointNum(); ++ii)
		{
			CString strIndex, strXPos, strYPos, strLine;
			PicControlPoint picPnt(m_samplePicData.getPoint(ii));
			ImagePoint point(picPnt.getXPos()-616/2, picPnt.getYPos()-808/2);
			double params[4] = {-1.85011491841446e-006, 
            4.88851183537935e-012, 
            5.21737401412096e-007, 
            -9.01547253250398e-006};
			SimplexVertex coefficient(4, params);
			ImagePoint origin(-2,8);
			CFUnDistort func(coefficient,point,origin);
         ImagePoint undistortPnt;
         point.undistort(func,&undistortPnt);
			//ImagePoint distortPnt = point.distortWithNoise(func,0.0,rand()); 
			strIndex.Format(_T("%d"),picPnt.getIndex());
			strXPos.Format(_T("%2.14f"),undistortPnt.getXPos());
			strYPos.Format(_T("%2.14f"),undistortPnt.getYPos());
			strLine = strIndex + _T(",") + strXPos + _T(",") + strYPos + _T("\n");
			file.WriteString(strLine);
		}
	}
	else
	{
		DBG_WARN("Open file fail in CDCCView::OnBnClickedBdistort");
		return;
	}
}

void CDCCView::OnBnClickedBloadsample()
{
	// TODO: Add your control notification handler code here
	CFileDialog fileDlg(TRUE, _T("*.ctr"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("CTR(*.ctr)|*.ctr||"), this);
	fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
	if ( fileDlg.DoModal() == IDOK )
	{
		m_sampleFilePath = fileDlg.GetPathName();
		PicDataInitUtil::initPicData(m_sampleFilePath,ImagePoint(m_nPicWidth/2.0,m_nPicHeight/2.0),m_samplePicData);
	}
}

void CDCCView::OnBnClickedBsave()
{
	// TODO: Add your control notification handler code here
	if (m_orientElemType == InteriorOrientElem)
	{
		// Write Interior Orientation Elements
		CStdioFile file;
		CString wfileName = m_imageFolderPath + _T("畸变参数.txt");
		if(file.Open(wfileName,CFile::modeCreate | CFile::modeWrite,NULL))
		{
			CString strParams[6] = {_T("x0 = "), _T("y0 = "), _T("C3 = "), _T("C5 = "), _T("P1 = "), _T("P2 = ")};
			for (int ii = 0; ii < 6; ++ii)
			{
				CString strTmp;
				double paramVal = ii > 1 ? m_distortParams.getEntry(ii-2) : (ii == 0 ? m_origin.getXPos() : m_origin.getYPos());
				if (ii < 2)
					strTmp.Format(_T("%2.4f\n"), paramVal);
				else
					strTmp.Format(_T("%2.14e\n"), paramVal);
				strParams[ii].Append(strTmp);
				file.WriteString(strParams[ii]);
			}
			if (m_bCheckDistort.GetCheck())
			{
				CString strErr;
				strErr.Format(_T("Average Error = %2.4f\n"), m_avgErr);
				file.WriteString(strErr);
			}
		}
		else
		{
			DBG_WARN("Open file fail in CDCCView::OnBnClickedBsave");
			return;
		}
	}
	else if (m_orientElemType == ExteriorOrientElem)
	{
		// Write Exterior Orientation Elements
		CStdioFile file;
		CString wfileName = m_imageFolderPath + _T("内外方位元素结果.txt");
		if(file.Open(wfileName,CFile::modeCreate | CFile::modeWrite,NULL))
		{
			CString strExParams[7] = {_T("Xs = "), _T("Ys = "), _T("Zs = "), _T("fi = "), _T("omiga = "), _T("kapa = "), _T("R = \n")};
			CString strInParams[7] = {_T("x0 = "), _T("y0 = "), _T("C3 = "), _T("C5 = "), _T("P1 = "), _T("P2 = "), _T("f = ")};
			
			CString strLineXYZ;
			for (int ii = 0; ii < 3; ++ii)
			{
				CString strTmp;
				double paramVal = m_exteriorParams(ii);
				strTmp.Format(_T("%6.2fmm "), paramVal);
				strExParams[ii].Append(strTmp);
				strLineXYZ.Append(strExParams[ii]);
			}
			strLineXYZ.Append(_T("\n"));
			file.WriteString(strLineXYZ); // Xs,Ys,Zs

			CString strLinefok;
			for (int ii = 3; ii < 6; ++ii)
			{
				double paramVal = m_exteriorParams.getEntry(ii);
				bool isNegative = paramVal < 0.0;
				Degree angle;
				MathUtil::RadToDeg(paramVal,angle);
				CString strDeg,strMin,strSec;
				if (isNegative)
					strDeg.Format(_T("-%-d^"),angle[0]);
				else
					strDeg.Format(_T("%-d^"),angle[0]);
				strMin.Format(_T("%-d\'"),angle[1]);
				strSec.Format(_T("%-d\" "),angle[2]);
				strExParams[ii].Append(strDeg); 
				strExParams[ii].Append(strMin); 
				strExParams[ii].Append(strSec);
				strLinefok.Append(strExParams[ii]);
			}
			strLinefok.Append(_T("\n"));
			file.WriteString(strLinefok); // φ,ω,κ

			{
				// Rotate Matrix
				file.WriteString(strExParams[6]);
				Matrix rotMatrix = MatrixUtil::generateRotateMatrix(m_exteriorParams(3),m_exteriorParams(4), m_exteriorParams(5));
				for (int kk = 0; kk < 3; ++kk)
				{
					CString strLineR;
					for (int jj = 0; jj < 3; ++jj)
					{
						CString strTmp;
						strTmp.Format(_T("\t%2.6f"),rotMatrix(kk,jj));
						strLineR.Append(strTmp);
					}
					strLineR.Append(_T("\n"));
					file.WriteString(strLineR);
				}
			}
			file.WriteString(_T("\n"));

			for (int ii = 0; ii < 6; ++ii)
			{
				CString strTmp;
				double paramVal = ii > 1 ? m_distortParams(ii-2) : (ii == 0 ? m_origin.getXPos() : m_origin.getYPos());
				if (ii < 2)
					strTmp.Format(_T("%2.4f\n"), paramVal);
				else
					strTmp.Format(_T("%2.14e\n"), paramVal);
				strInParams[ii].Append(strTmp);
				file.WriteString(strInParams[ii]); // x0,y0,c3,c5,p1,p2
			}
			CString strf;
			strf.Format(_T("%4.2f"),m_exteriorParams(6));
			strInParams[6].Append(strf);
			file.WriteString(strInParams[6]); // f
		}
		else
		{
			DBG_WARN("Open file fail in CDCCView::OnBnClickedBsave");
			return;
		}
	}
	else
	{
		DBG_WARN("Unknown OrientElemType is found in CDCCView::OnBnClickedBsave");
		return;
	}

	m_bSave.EnableWindow(FALSE);
}

void CDCCView::OnBnClickedCheckdistort()
{
	// TODO: Add your control notification handler code here
	if (m_bCheckDistort.GetCheck())
	{
		m_bLoadSample.EnableWindow(TRUE);
		m_bDistort.EnableWindow(TRUE);
      m_bCreateSample.EnableWindow(TRUE);
	} 
	else
	{
		m_bLoadSample.EnableWindow(FALSE);
		m_bDistort.EnableWindow(FALSE);
      m_bCreateSample.EnableWindow(FALSE);
	}
}

void CDCCView::OnBnClickedBcreatesample()
{
   // TODO: Add your control notification handler code here
   // Create eight lines, every of which has 50 points.
   // These lines include: 1) x + y = -150; 2) x + y = 150;
   // 3) x - y = 150; 4) x - y = -150; 5) x + y = -400;
   // 6) x + y = 400; 7) x - y = 400; 8) x - y = -400
   CStdioFile file;
   CString wfileName = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if(file.Open(wfileName,CFile::modeCreate | CFile::modeWrite,NULL))
   {
      const int lineN = 8;
      const int pointN = 50;
      const double stepX = 8;
      double startPointX[8] = {-275, -125, -125, -275, -400, 0, 0, -400};
      double startPointY[8] = {125, 275, -275, -125, 0, 400, -400, 0};
      
      CString totalPoint;
      totalPoint.Format(_T("%d\n"),lineN*pointN);
      file.WriteString(totalPoint); 

      CString strLine, strX, strY, strIndex;
      for (int ii = 0; ii < lineN; ++ii)
      {
         double stepY = (ii%4 < 2) ? -8 : 8;
         for (int jj = 0; jj < pointN; ++jj)
         {
            int index = (1+ii)*100 + jj + 1;
            double x = startPointX[ii] + jj * stepX;
            double y = startPointY[ii] + jj * stepY;
            strIndex.Format(_T("%d"),index);
            strX.Format(_T("%2.14f"),x);
            strY.Format(_T("%2.14f"),y);
            strLine = strIndex + _T(",") + strX + _T(",") + strY + _T("\n");
            file.WriteString(strLine);
         }
      }
   }
   else
   {
      DBG_WARN("Open file fail in CDCCView::OnBnClickedBcreatesample");
      return;
   }
}

