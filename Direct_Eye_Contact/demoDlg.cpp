#include "stdafx.h"
#include "demoDlg.h"


CdemoDlg::CdemoDlg() : CDialogEx(CdemoDlg::IDD)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON);
}


BOOL CdemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	m_font.CreatePointFont(100, L"Arial Bold");

	GetDlgItem(IDC_START)->SetFont(&m_font);

	cv::namedWindow("camera view", CV_WINDOW_AUTOSIZE);
	HWND hWnd = (HWND)cvGetWindowHandle("camera view");
	HWND hParent = ::GetParent(hWnd);
	::SetParent(hWnd, GetDlgItem(IDC_DEMO)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	// IN CASE USER HAS OPENED THE CAMERAS
	if (!cap_L.isOpened())
	{
		cap_L = cv::VideoCapture(0);
		//cap_R = cv::VideoCapture(1);
		if (!cap_L.isOpened())
		{
			AfxMessageBox(_T("UNABLE TO OPEN CAMERAS"));
			return FALSE;
		}
	}

	SetTimer(1, 10, NULL);

	return TRUE;
}


void CdemoDlg::DoDataExchange(CDataExchange * pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CdemoDlg, CDialogEx)
	ON_BN_CLICKED(IDC_START, &CdemoDlg::OnBnClickedStart)
	ON_WM_TIMER()
END_MESSAGE_MAP()


void CdemoDlg::OnBnClickedStart()
{
	if (!if_synth)
		if_synth = TRUE;
	else
		if_synth = FALSE;
}


void CdemoDlg::OnTimer(UINT_PTR nIDEvent)
{
	cap_L >> cap_mat_L;
	//cap_R >> cap_mat_R;

	if (if_synth)
	{

	}
	else
	{
		CRect rect;
		GetDlgItem(IDC_DEMO)->GetClientRect(&rect);
		cv::resize(cap_mat_L, cap_mat_L, cv::Size(rect.Width(), rect.Height()));
		cv::imshow("camera view", cap_mat_L);
	}

	CDialogEx::OnTimer(nIDEvent);
}
