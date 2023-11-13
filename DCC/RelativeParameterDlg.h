#pragma once
#include "afxwin.h"


// CRelativeParameterDlg dialog

class CRelativeParameterDlg : public CDialog
{
	DECLARE_DYNAMIC(CRelativeParameterDlg)

public:
	CRelativeParameterDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CRelativeParameterDlg();

// Dialog Data
	enum { IDD = IDD_RELATIVE_PARAM_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:
   afx_msg void OnBnClickedOk();
   afx_msg void OnBnClickedOpenCompareFile();
   afx_msg void OnBnClickedOpenBaseFile();
   afx_msg void OnBnClickedRpdlgCancel();

private:
   CEdit             m_editCompareFileName;
   CEdit             m_editBaseFileName;
   CString           m_compareFileName;
   CString           m_baseFileName;
   CString           m_savePath;   
};
