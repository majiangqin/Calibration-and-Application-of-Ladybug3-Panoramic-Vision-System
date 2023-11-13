// RelativeParameterDlg.cpp : implementation file
//

#include "stdafx.h"
#include "DCC.h"
#include "RelativeParameterDlg.h"
#include "PicDataInitUtil.h"
#include "Matrix.h"
#include "CalibrationUtil.h"
#include "MathUtil.h"
#include "Warning.h"


// CRelativeParameterDlg dialog

IMPLEMENT_DYNAMIC(CRelativeParameterDlg, CDialog)

CRelativeParameterDlg::CRelativeParameterDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRelativeParameterDlg::IDD, pParent)
{

}

CRelativeParameterDlg::~CRelativeParameterDlg()
{
}

void CRelativeParameterDlg::DoDataExchange(CDataExchange* pDX)
{
   CDialog::DoDataExchange(pDX);
   DDX_Control(pDX, IDC_COMPARE_FILE, m_editCompareFileName);
   DDX_Control(pDX, IDC_BASE_FILE, m_editBaseFileName);
}


BEGIN_MESSAGE_MAP(CRelativeParameterDlg, CDialog)
   ON_BN_CLICKED(IDC_RPDLG_OK, &CRelativeParameterDlg::OnBnClickedOk)
   ON_BN_CLICKED(IDC_OPEN_COMPARE_FILE, &CRelativeParameterDlg::OnBnClickedOpenCompareFile)
   ON_BN_CLICKED(IDC_OPEN_BASE_FILE, &CRelativeParameterDlg::OnBnClickedOpenBaseFile)
   ON_BN_CLICKED(IDC_RPDLG_CANCEL, &CRelativeParameterDlg::OnBnClickedRpdlgCancel)
END_MESSAGE_MAP()


// CRelativeParameterDlg message handlers

void CRelativeParameterDlg::OnBnClickedOk()
{
   // TODO: Add your control notification handler code here
   SimplexVertex baseParams(3), compareParams(3), relativeParams(3);
   Matrix baseRotateMatrix(3,3), compareRotateMatrix(3,3), relativeRotateMatrix(3,3);
   PicDataInitUtil::loadCameraOutParams(m_baseFileName,baseParams,baseRotateMatrix);
   PicDataInitUtil::loadCameraOutParams(m_compareFileName,compareParams,compareRotateMatrix);
   RelativeCalibrationUtil::calculate(baseParams,compareParams,baseRotateMatrix,compareRotateMatrix,&relativeParams,&relativeRotateMatrix);
   
   double fi, omiga, kapa;
   MatrixUtil::computeRotateAngle(relativeRotateMatrix,&fi,&omiga,&kapa);

   // Write Relative Exterior Orientation Elements
   CStdioFile file;
   CString wfileName = m_savePath + _T("相关外参数.txt");
   if(file.Open(wfileName,CFile::modeCreate | CFile::modeWrite,NULL))
   {
      CString strExParams[7] = {_T("Xs = "), _T("Ys = "), _T("Zs = "), _T("fi = "), _T("omiga = "), _T("kapa = "), _T("R = \n")};
      
      CString strLineXYZ;
      for (int ii = 0; ii < 3; ++ii)
      {
         CString strTmp;
         double paramVal = relativeParams(ii);
         strTmp.Format(_T("%6.4fmm "), paramVal);
         strExParams[ii].Append(strTmp);
         strLineXYZ.Append(strExParams[ii]);
      }
      strLineXYZ.Append(_T("\n"));
      file.WriteString(strLineXYZ); // Xs,Ys,Zs

      CString strLinefok;
      double angles[3] = {fi, omiga, kapa};
      for (int ii = 3; ii < 6; ++ii)
      {
         double paramVal = angles[ii-3];
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
         for (int kk = 0; kk < 3; ++kk)
         {
            CString strLineR;
            for (int jj = 0; jj < 3; ++jj)
            {
               CString strTmp;
               strTmp.Format(_T("\t%2.6f"),relativeRotateMatrix(kk,jj));
               strLineR.Append(strTmp);
            }
            strLineR.Append(_T("\n"));
            file.WriteString(strLineR);
         }
      }
      file.WriteString(_T("\n"));
   }
   else
   {
      DBG_WARN("Open file fail in CRelativeParameterDlg::OnBnClickedOk");
      return;
   }

   OnOK();
}

void CRelativeParameterDlg::OnBnClickedOpenCompareFile()
{
   // TODO: Add your control notification handler code here
   CFileDialog fileDlg(TRUE, _T("*.txt"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Camera Parameter File(*.txt)|*.txt||"), this);
   fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if ( fileDlg.DoModal() == IDOK )
   {
      m_compareFileName = fileDlg.GetPathName();
      m_savePath = m_compareFileName.Left(m_compareFileName.Find(fileDlg.GetFileName()));
      m_editCompareFileName.SetWindowText(m_compareFileName);
   }
}

void CRelativeParameterDlg::OnBnClickedOpenBaseFile()
{
   // TODO: Add your control notification handler code here
   CFileDialog fileDlg(TRUE, _T("*.txt"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Camera Parameter File(*.txt)|*.txt||"), this);
   fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if ( fileDlg.DoModal() == IDOK )
   {
      m_baseFileName = fileDlg.GetPathName();
      m_editBaseFileName.SetWindowText(m_baseFileName);
   }
}

void CRelativeParameterDlg::OnBnClickedRpdlgCancel()
{
   // TODO: Add your control notification handler code here
   OnCancel();
}
