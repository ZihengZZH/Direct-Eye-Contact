// Direct_Eye_Contact.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "resource.h"
#include "calib.h"
#include "faceDepth.h"
#include "mainDlg.h"
#include "demoDlg.h"
#include "CSplashDlg.h"


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
    /* SPLASH DIALOG */
    CSplashDlg splash;
    INT_PTR nRet = splash.DoModal();
    Sleep(4000);
    CloseWindow(splash);
    DestroyWindow(splash);

    /* DEFAULT OPEN MAIN DIALOG */
    CmainDlg main;
    m_pMainWnd = &main;
    main.DoModal();

    return TRUE;
}


int CMAINApp::ExitInstance()
{
    return 0;
}


CMAINApp theApp;
