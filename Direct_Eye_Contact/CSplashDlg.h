#pragma once


// CSplashDlg dialog

#include "resource.h"

class CSplashDlg : public CDialogEx
{
    DECLARE_DYNAMIC(CSplashDlg)

public:
    CSplashDlg();
    virtual ~CSplashDlg();
    enum { IDD = IDD_SPLASH };


protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
};
