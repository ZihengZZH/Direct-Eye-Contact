#include "stdafx.h"
#include "mainDlg.h"


CmainDlg::CmainDlg() : CDialogEx(CmainDlg::IDD)
{
}

BOOL CmainDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	cv::namedWindow("left view", CV_WINDOW_AUTOSIZE);
	HWND hWnd_L = (HWND)cvGetWindowHandle("left view");
	HWND hParent = ::GetParent(hWnd_L);
	::SetParent(hWnd_L, GetDlgItem(IDC_CAM_L)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	cv::namedWindow("right view", CV_WINDOW_AUTOSIZE);
	HWND hWnd_R = (HWND)cvGetWindowHandle("right view");
	hParent = ::GetParent(hWnd_R);
	::SetParent(hWnd_R, GetDlgItem(IDC_CAM_R)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	cv::namedWindow("depth map", CV_WINDOW_AUTOSIZE);
	HWND hWnd_D = (HWND)cvGetWindowHandle("depth map");
	hParent = ::GetParent(hWnd_D);
	::SetParent(hWnd_D, GetDlgItem(IDC_DEPTH)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	cv::namedWindow("synthesis", CV_WINDOW_AUTOSIZE);
	HWND hWnd_S = (HWND)cvGetWindowHandle("synthesis");
	hParent = ::GetParent(hWnd_S);
	::SetParent(hWnd_S, GetDlgItem(IDC_SYNTH)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	cap_L = cv::VideoCapture(0);
	cap_R = cv::VideoCapture(1);
	if (!cap_L.isOpened() || !cap_R.isOpened())
	{
		AfxMessageBox(_T("UNABLE TO OPEN CAMERAS"));
		return FALSE;
	}
	mat_depth_standby = cv::imread("..\\Direct_Eye_Contact\\image\\standby_depth.png");
	mat_synth_standby = cv::imread("..\\Direct_Eye_Contact\\image\\standby_synth.png");

	face.readParameter();

	return TRUE;
}

void CmainDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CmainDlg, CDialogEx)
	ON_BN_CLICKED(ID_OPEN, &CmainDlg::OnBnClickedOpen)
	ON_WM_TIMER()
	ON_BN_CLICKED(ID_FACIAL, &CmainDlg::OnBnClickedFacial)
	ON_BN_CLICKED(ID_INFO, &CmainDlg::OnBnClickedInfo)
	ON_BN_CLICKED(ID_DEPTH, &CmainDlg::OnBnClickedDepth)
	ON_BN_CLICKED(ID_SYNTH, &CmainDlg::OnBnClickedSynth)
	ON_BN_CLICKED(ID_CLOSE, &CmainDlg::OnBnClickedClose)
END_MESSAGE_MAP()

void CmainDlg::OnBnClickedOpen()
{
	
	/*if (false)
	{
		std::string pic_path = "./test/test.jpg";
		cv::Mat image = cv::imread(pic_path);
		cv::Mat imagedst; // change the size to fit
	}

	CRect rect;
	GetDlgItem(IDC_CAM_L)->GetClientRect(&rect);
	cv::Rect dst(rect.left, rect.top, rect.right, rect.bottom);
	cv::resize(image, imagedst, cv::Size(rect.Width(), rect.Height()));

	// OPTIONAL FOR PIC CONTROL SIZE
	{
		CEdit* pBoxOne;
		pBoxOne = (CEdit*)GetDlgItem(IDC_CONSOLE);
		CString str;
		str.Format(_T("Width %3d Height %3d"), rect.Width(), rect.Height());
		pBoxOne->SetWindowText(str);
		str.ReleaseBuffer();
	}*/

	SetTimer(1, 10, NULL);
}


void CmainDlg::OnTimer(UINT_PTR nIDEvent)
{
	
	cap_L >> cap_mat_L;
	cap_R >> cap_mat_R;

	cv::undistort(cap_mat_L, face.imgLeft_col, face.M1, face.D1);
	cv::undistort(cap_mat_R, face.imgRight_col, face.M2, face.D2);

	//face.imgLeft_col = cap_mat_L;
	//face.imgRight_col = cap_mat_R;

	if (if_landmarks)
	{
		cap_mat_L = face.facialLandmarkVis(true);
		cap_mat_R = face.facialLandmarkVis(false);
	}

	if (if_info)
	{
		if (face.facialLandmark(true) && face.facialLandmark(false))
		{
			face.levelDepthVis(cap_mat_L, true);
		}
	}

	if (if_depth)
	{
		if (face.facialLandmark(true) && face.facialLandmark(false))
		{
			cv::Mat depth_mat;
			cap_mat_L.copyTo(depth_mat);
			face.calDepth();
			face.levelDepthVis(depth_mat, false);
			cv::imshow("depth map", depth_mat);
		}
	}
	else
	{
		CRect rect;
		GetDlgItem(IDC_DEPTH)->GetClientRect(&rect);
		cv::resize(mat_depth_standby, mat_depth_standby, cv::Size(rect.Width(), rect.Height()));
		cv::imshow("depth map", mat_depth_standby);
	}
	
	if (if_synth)
	{

	}
	else
	{
		CRect rect;
		GetDlgItem(IDC_SYNTH)->GetClientRect(&rect);
		cv::resize(mat_synth_standby, mat_synth_standby, cv::Size(rect.Width(), rect.Height()));
		cv::imshow("synthesis", mat_synth_standby);
	}

	cv::imshow("left view", cap_mat_L);
	cv::imshow("right view", cap_mat_R);

	CDialogEx::OnTimer(nIDEvent);
}


void CmainDlg::OnBnClickedFacial()
{
	if_landmarks = TRUE;
}


void CmainDlg::OnBnClickedInfo()
{
	if_info = TRUE;
}


void CmainDlg::OnBnClickedDepth()
{
	if_depth = TRUE;
}


void CmainDlg::OnBnClickedSynth()
{
	if_synth = TRUE;
}


void CmainDlg::OnBnClickedClose()
{	
	if_landmarks = FALSE;
	if_info = FALSE;
	if_depth = FALSE;
	if_synth = FALSE;
}




