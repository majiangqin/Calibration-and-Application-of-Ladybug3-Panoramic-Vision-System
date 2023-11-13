// DCCDoc.cpp : implementation of the CDCCDoc class
//

#include "stdafx.h"
#include "DCC.h"

#include "DCCDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CDCCDoc

IMPLEMENT_DYNCREATE(CDCCDoc, CDocument)


// CDCCDoc construction/destruction

CDCCDoc::CDCCDoc()
{
	// TODO: add one-time construction code here

}

CDCCDoc::~CDCCDoc()
{
}

BOOL CDCCDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CDCCDoc serialization

void CDCCDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CDCCDoc diagnostics

#ifdef _DEBUG
void CDCCDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CDCCDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG




