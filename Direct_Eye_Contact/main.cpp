// Direct_Eye_Contact.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "resource.h"
#include "calib.h"
#include "faceDepth.h"
#include "mainDlg.h"


/* CONSOLE APPLICATION TO MFC DIALOG APPLICATION */
// Property -> General -> Use of MFC (Windows Libraries -> Shared DLL)
// Property -> Linker -> System -> SubSystem (Console -> Windows)
// Property -> C/C++ -> Advanced -> Show includes (IN CASE HEADERS ERRPR)


class CMAINApp : public CWinApp
{
public:
	CMAINApp();

	virtual BOOL InitInstance();
	virtual int ExitInstance();
};


CMAINApp::CMAINApp()
{
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_ALL_ASPECTS;

#ifdef _MANAGED
	System::Windows::Forms::Application::SetUnhandledExceptionMode(System::Windows::Forms::UnhandledExceptionMode::ThrowException);
#endif

	SetAppID(_T("Eye_Contact.AppID.2.0"));
}


BOOL CMAINApp::InitInstance()
{
	CmainDlg dlg;
	m_pMainWnd = &dlg;
	dlg.DoModal();

	return TRUE;
}


int CMAINApp::ExitInstance()
{
	return 0;
}


CMAINApp theApp;