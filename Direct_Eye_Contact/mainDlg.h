#pragma once

#include "resource.h"

/* CLASS OF MFC DIALOG */
// this class is responsible for MFC dialog design

class CmainDlg :public CDialogEx
{
public:
	CmainDlg();
	enum { IDD = IDD_MAIN };
	BOOL OnInitDialog();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);

protected:
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedOpen();
};

