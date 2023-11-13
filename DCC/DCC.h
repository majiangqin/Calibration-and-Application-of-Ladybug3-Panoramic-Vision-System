// DCC.h : main header file for the DCC application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CDCCApp:
// See DCC.cpp for the implementation of this class
//

class CDCCApp : public CWinApp
{
public:
	CDCCApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
   afx_msg void OnRelativeParameter();
   afx_msg void OnCtrInvCompute();
   afx_msg void OnSetDefaultPath();

public:
   CString        m_strFolderPath;
};

extern CDCCApp theApp;