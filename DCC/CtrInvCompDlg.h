#pragma once
#include "afxwin.h"


// CCtrInvCompDlg dialog

class CCtrInvCompDlg : public CDialog
{
	DECLARE_DYNAMIC(CCtrInvCompDlg)

public:
	CCtrInvCompDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CCtrInvCompDlg();

// Dialog Data
	enum { IDD = IDD_CTR_INVERSE_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:
   afx_msg void OnBnClickedOpenCamera1();
   afx_msg void OnBnClickedOpenCamera2();
   afx_msg void OnBnClickedOpenCamera3();
   afx_msg void OnBnClickedOk();
   afx_msg void OnBnClickedCpicdlgCancel();

private:
   CEdit          m_editCameraFolder1;
   CEdit          m_editCameraFolder2;
   CEdit          m_editCameraFolder3;
   CString        m_cameraFolder1;
   CString        m_cameraFolder2;
   CString        m_cameraFolder3;
   CString        m_picFile1;
   CString        m_picFile2;
   CString        m_picFile3;
   bool           m_hasFile1;
   bool           m_hasFile2;
   bool           m_hasFile3;
};
