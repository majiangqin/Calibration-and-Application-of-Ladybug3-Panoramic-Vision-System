// DCCView.h : interface of the CDCCView class
//


#pragma once

#include "PicData.h"
#include "afxwin.h"
#include "CalibrationUtil.h"
#include "Matrix.h"


class CDCCView : public CFormView
{
protected: // create from serialization only
	CDCCView();
	DECLARE_DYNCREATE(CDCCView)

public:
	enum{ IDD = IDD_DCC_FORM };

// Attributes
public:
	CDCCDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnInitialUpdate(); // called first time after construct

// Implementation
public:
	virtual ~CDCCView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg void OnCbnSelchangeObjFuncCombo();
	afx_msg void OnBnClickedBloadimage();
	afx_msg void OnBnClickedBcalibration();
	afx_msg void OnBnClickedBdistort();
	afx_msg void OnBnClickedBloadsample();
	afx_msg void OnBnClickedBsave();
	afx_msg void OnBnClickedCheckdistort();
   afx_msg void OnBnClickedBcreatesample();

public:
	CButton						m_bCheckDistort;
	CButton						m_bLoadSample;
	CButton						m_bDistort;
	CButton						m_bCalibration;
	CButton						m_bSave;
   CButton                 m_bCreateSample;
	CComboBox					m_objFuncComboBox;

private:
	PicData						m_realPicData;
	PicData						m_samplePicData;
	ObjectData					m_objectData;
	CString						m_imageFolderPath;
	CString						m_sampleFilePath;
	SimplexVertex				m_distortParams; // C3,C5,P1,P2
	SimplexVertex				m_exteriorParams; // Xs,Ys,Zs,¦Õ,¦Ø,¦Ê,f
	ImagePoint					m_origin; // x0,y0
	double						m_avgErr;
	int							m_lineN;
	int							m_distortParamN;
	int							m_nPicWidth;
	int							m_nPicHeight;
	ObjectMethod				m_funcType;
	OrientElemType				m_orientElemType;
};

#ifndef _DEBUG  // debug version in DCCView.cpp
inline CDCCDoc* CDCCView::GetDocument() const
   { return reinterpret_cast<CDCCDoc*>(m_pDocument); }
#endif

