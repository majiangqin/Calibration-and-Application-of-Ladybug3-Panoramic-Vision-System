// DCCDoc.h : interface of the CDCCDoc class
//


#pragma once


class CDCCDoc : public CDocument
{
protected: // create from serialization only
	CDCCDoc();
	DECLARE_DYNCREATE(CDCCDoc)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CDCCDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

};


