// CtrInvCompDlg.cpp : implementation file
//

#include "stdafx.h"
#include "DCC.h"
#include "CtrInvCompDlg.h"
#include "InverseControlPointUtil.h"
#include "PicDataInitUtil.h"
#include "PicData.h"
#include "Matrix.h"
#include "Warning.h"


// CCtrInvCompDlg dialog

IMPLEMENT_DYNAMIC(CCtrInvCompDlg, CDialog)

CCtrInvCompDlg::CCtrInvCompDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CCtrInvCompDlg::IDD, pParent)
{
   m_hasFile1 = false;
   m_hasFile2 = false;
   m_hasFile3 = false;
}

CCtrInvCompDlg::~CCtrInvCompDlg()
{
}

void CCtrInvCompDlg::DoDataExchange(CDataExchange* pDX)
{
   CDialog::DoDataExchange(pDX);
   DDX_Control(pDX, IDC_CAMERA1, m_editCameraFolder1);
   DDX_Control(pDX, IDC_CAMERA2, m_editCameraFolder2);
   DDX_Control(pDX, IDC_CAMERA3, m_editCameraFolder3);
}


BEGIN_MESSAGE_MAP(CCtrInvCompDlg, CDialog)
   ON_BN_CLICKED(IDC_CPICDLG_OK, &CCtrInvCompDlg::OnBnClickedOk)
   ON_BN_CLICKED(IDC_CPICDLG_CANCEL, &CCtrInvCompDlg::OnBnClickedCpicdlgCancel)
   ON_BN_CLICKED(IDC_OPEN_CAMERA1, &CCtrInvCompDlg::OnBnClickedOpenCamera1)
   ON_BN_CLICKED(IDC_OPEN_CAMERA2, &CCtrInvCompDlg::OnBnClickedOpenCamera2)
   ON_BN_CLICKED(IDC_OPEN_CAMERA3, &CCtrInvCompDlg::OnBnClickedOpenCamera3)
END_MESSAGE_MAP()


// CCtrInvCompDlg message handlers

void CCtrInvCompDlg::OnBnClickedOpenCamera1()
{
   // TODO: Add your control notification handler code here
   CFileDialog fileDlg(TRUE, _T("*.bmp"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Camera Folder(*.bmp)|*.bmp||"), this);
   fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if ( fileDlg.DoModal() == IDOK )
   {
      m_picFile1 = fileDlg.GetPathName();
      m_cameraFolder1 = m_picFile1.Left(m_picFile1.Find(fileDlg.GetFileName()));
      m_editCameraFolder1.SetWindowText(m_cameraFolder1);
      m_hasFile1 = true;
   }
}

void CCtrInvCompDlg::OnBnClickedOpenCamera2()
{
   // TODO: Add your control notification handler code here
   CFileDialog fileDlg(TRUE, _T("*.bmp"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Camera Folder(*.bmp)|*.bmp||"), this);
   fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if ( fileDlg.DoModal() == IDOK )
   {
      m_picFile2 = fileDlg.GetPathName();
      m_cameraFolder2 = m_picFile2.Left(m_picFile2.Find(fileDlg.GetFileName()));
      m_editCameraFolder2.SetWindowText(m_cameraFolder2);
      m_hasFile2 = true;
   }
}

void CCtrInvCompDlg::OnBnClickedOpenCamera3()
{
   // TODO: Add your control notification handler code here
   CFileDialog fileDlg(TRUE, _T("*.bmp"), NULL, OFN_EXPLORER|OFN_FILEMUSTEXIST, _T("Camera Folder(*.bmp)|*.bmp||"), this);
   fileDlg.m_ofn.lpstrInitialDir = dynamic_cast<const CDCCApp* >(AfxGetApp())->m_strFolderPath;
   if ( fileDlg.DoModal() == IDOK )
   {
      m_picFile3 = fileDlg.GetPathName();
      m_cameraFolder3 = m_picFile3.Left(m_picFile3.Find(fileDlg.GetFileName()));
      m_editCameraFolder3.SetWindowText(m_cameraFolder3);
      m_hasFile3 = true;
   }
}

void CCtrInvCompDlg::OnBnClickedOk()
{
   // TODO: Add your control notification handler code here
   const int distortParamN = 4;
   const int compareDataN = (m_hasFile1 ? 1 : 0) + (m_hasFile2 ? 1 : 0) + (m_hasFile3 ? 1 : 0);
   bool hasFile[3] = {m_hasFile1,m_hasFile2,m_hasFile3};

   Matrix* pMatrix = new Matrix[compareDataN]; // rotation matrix
   SimplexVertex* pExteriorParams = new SimplexVertex[compareDataN]; // Xs, Ys, Zs
   SimplexVertex* pInteriorParams = new SimplexVertex[compareDataN]; // x0, y0, c3, c5, p1, p2, f
   PicData* pPicData = new PicData[compareDataN];
   ObjectData invCtrData;

   CString picFile[3] = {m_picFile1,m_picFile2,m_picFile3};
   CString cameraFolder[3] = {m_cameraFolder1,m_cameraFolder2,m_cameraFolder3};
   int picWidth[3]; int picHeight[3];
   for (int ii = 0, jj = 0; ii < compareDataN; ++jj)
   {
      if (hasFile[jj])
      {
         CString picDataFile = picFile[jj].Left(picFile[jj].Find('.')+1) + _T("ctr");
         CString paramFile = cameraFolder[jj] + _T("内外方位元素结果.txt");
         PicDataInitUtil::loadImageInfo(picFile[jj],&picWidth[jj],&picHeight[jj]);
         PicDataInitUtil::initPicData(picDataFile,ImagePoint(picWidth[jj]/2.0,picHeight[jj]/2.0),pPicData[ii]);
         PicDataInitUtil::loadCameraOutParams(paramFile,pExteriorParams[ii],pMatrix[ii]);
         PicDataInitUtil::loadCameraInParams(paramFile,pInteriorParams[ii],distortParamN);
         ++ ii;
      }
   }

   InverseControlPointUtil::calculate(pPicData,pExteriorParams,pInteriorParams,pMatrix,compareDataN,&invCtrData);
   
   // Write inverse control points into file
   CString savePath = m_cameraFolder1 + _T("反算控制点坐标.txt");
   CStdioFile file;
   if(file.Open(savePath,CFile::modeCreate | CFile::modeWrite,NULL))
   {
      CString headStr[4] = {_T("第一列是点号，单位：毫米\n"), _T("第二列是X坐标，朝南为正向\n"), 
                            _T("第三列是Y坐标，朝上为正向\n"), _T("第四列为Z坐标，朝前(西)为正向\n")};
      for (int ii = 0; ii < 4; ++ii)
      {
         file.WriteString(headStr[ii]);
      }
      CString strPointN;
      strPointN.Format(_T("%d\n"),invCtrData.getPointNum());
      for (int ii = 0; ii < invCtrData.getPointNum(); ++ii)
      {
         int index = invCtrData.getPoint(ii).getIndex();
         double x = invCtrData.getPoint(ii).getXPos();
         double y = invCtrData.getPoint(ii).getYPos();
         double z = invCtrData.getPoint(ii).getZPos();

         CString strLine;
         CString strTmp[4];
         strTmp[0].Format(_T("%d\t"),index);
         strTmp[1].Format(_T("%.4f\t"),x);
         strTmp[2].Format(_T("%.4f\t"),y);
         strTmp[3].Format(_T("%.4f\n"),z);
         for (int jj = 0; jj < 4; ++jj)
         {
            strLine.Append(strTmp[jj]);
         }
         file.WriteString(strLine);
      }
   }
   else
   {
      DBG_WARN("Open file fail in CCtrInvCompDlg::OnBnClickedOk");
      return;
   }

   OnOK();
}

void CCtrInvCompDlg::OnBnClickedCpicdlgCancel()
{
   // TODO: Add your control notification handler code here
   OnCancel();
}
