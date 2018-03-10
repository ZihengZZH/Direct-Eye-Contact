#include "stdafx.h"
#include "mainDlg.h"


CmainDlg::CmainDlg() : CDialogEx(CmainDlg::IDD)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON);
}


BOOL CmainDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	m_font.CreatePointFont(150, L"Arial");
	m_font_small.CreatePointFont(100, L"Arial");

	GetDlgItem(ID_OPEN)->SetFont(&m_font);
	GetDlgItem(ID_DEPTH)->SetFont(&m_font);
	GetDlgItem(ID_SYNTH)->SetFont(&m_font);
	GetDlgItem(ID_CLOSE)->SetFont(&m_font);

	GetDlgItem(ID_FACIAL)->SetFont(&m_font_small);
	GetDlgItem(ID_INFO)->SetFont(&m_font_small);
	GetDlgItem(ID_RECORD)->SetFont(&m_font_small);
	GetDlgItem(ID_CALIB)->SetFont(&m_font_small);

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

	pBoxOne = (CEdit*)GetDlgItem(IDC_CONSOLE);

	m_logo.Load(_T("..\\Direct_Eye_Contact\\image\\logo.jpg"));
	m_Logo.SetBitmap(HBITMAP(m_logo));
	//int dimx = 300, dimy = 100;

	mat_depth_standby = cv::imread("..\\Direct_Eye_Contact\\image\\standby_depth.png");
	mat_synth_standby = cv::imread("..\\Direct_Eye_Contact\\image\\standby_synth.png");

	calib.readStringList();
	if (face.readParameter())
		already_calib = TRUE;

	return TRUE;
}


void CmainDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LOGO, m_Logo);
	DDX_Control(pDX, IDC_ICON, m_icon);
	DDX_Control(pDX, IDC_INFO, m_info);
}


BEGIN_MESSAGE_MAP(CmainDlg, CDialogEx)
	ON_BN_CLICKED(ID_OPEN, &CmainDlg::OnBnClickedOpen)
	ON_WM_TIMER()
	ON_BN_CLICKED(ID_FACIAL, &CmainDlg::OnBnClickedFacial)
	ON_BN_CLICKED(ID_INFO, &CmainDlg::OnBnClickedInfo)
	ON_BN_CLICKED(ID_DEPTH, &CmainDlg::OnBnClickedDepth)
	ON_BN_CLICKED(ID_SYNTH, &CmainDlg::OnBnClickedSynth)
	ON_BN_CLICKED(ID_CLOSE, &CmainDlg::OnBnClickedClose)
	ON_BN_CLICKED(ID_RECORD, &CmainDlg::OnBnClickedRecord)
	ON_BN_CLICKED(ID_CALIB, &CmainDlg::OnBnClickedCalib)
	ON_BN_CLICKED(IDC_DEMO, &CmainDlg::OnBnClickedDemo)
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
	
	// IN CASE USER HAS OPENED THE CAMERAS
	if (!cap_L.isOpened() && !cap_R.isOpened())
	{
		cap_L = cv::VideoCapture(0);
		cap_R = cv::VideoCapture(1);
		if (!cap_L.isOpened() || !cap_R.isOpened())
		{
			AfxMessageBox(_T("UNABLE TO OPEN CAMERAS"));
			return;
		}
	}
	
	SetTimer(1, 10, NULL);
}


void CmainDlg::OnTimer(UINT_PTR nIDEvent)
{
	
	cap_L >> cap_mat_L;
	cap_R >> cap_mat_R;

	if (if_record)
	{
		cap_mat_L.copyTo(calib.camera_matL);
		cap_mat_R.copyTo(calib.camera_matR);
		calib.saveImage();

		m_str.Format(_T("Images saved to files NO _%u_"), calib.time);
		pBoxOne->SetWindowText(m_str);
		m_str.ReleaseBuffer();
		if_record = FALSE;
	}

	if (if_calib)
	{
		m_str.Format(_T("Stereo Calibration in progress..."));
		pBoxOne->SetWindowText(m_str);
		m_str.ReleaseBuffer();

		if (true)
		{
			calib.stereoCalib();
			m_str.Format(_T("Stereo Calibration completed"));
			pBoxOne->SetWindowText(m_str);
			m_str.ReleaseBuffer();
		}

		if (face.readParameter())
		{
			if_calib = FALSE;
			already_calib = TRUE;
		}
	}

	if (already_calib)
	{
		cv::undistort(cap_mat_L, cap_mat_L_calib, face.M1, face.D1);
		cv::undistort(cap_mat_R, cap_mat_R_calib, face.M2, face.D2);
		cap_mat_L_calib.copyTo(face.imgLeft_col);
		cap_mat_R_calib.copyTo(face.imgRight_col);

		if (if_landmarks)
		{
			cap_mat_L_calib = face.facialLandmarkVis(true);
			cap_mat_R_calib = face.facialLandmarkVis(false);
		}

		if (if_info)
		{
			if (face.facialLandmark(true) && face.facialLandmark(false))
			{
				face.levelDepthVis(cap_mat_L_calib, true);
			}
		}

		if (if_depth)
		{
			if (face.facialLandmark(true) && face.facialLandmark(false))
			{
				cv::Mat depth_mat;
				cap_mat_L_calib.copyTo(depth_mat);
				face.calDepth();
				if (depth_method == USE_LEVEL)
					face.levelDepthVis(depth_mat, false);
				if (depth_method == USE_VORONOI)
					face.delaunayDepthVis(depth_mat);
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
		
		cv::imshow("left view", cap_mat_L_calib);
		cv::imshow("right view", cap_mat_R_calib);
	}
	else
	{
		m_str.Format(_T("PLEASE RUN CALIBRATION FIRST"));
		pBoxOne->SetWindowText(m_str);
		m_str.ReleaseBuffer();

		cv::imshow("left view", cap_mat_L);
		cv::imshow("right view", cap_mat_R);
	}

	CDialogEx::OnTimer(nIDEvent);
}


void CmainDlg::OnBnClickedRecord()
{
	if_record = TRUE;
	if (calib.time == calib.times)
	{
		if_record = FALSE;
		m_str.Format(_T("Images have been stored. Please begin stereo calibration"));
		pBoxOne->SetWindowText(m_str);
		m_str.ReleaseBuffer();
	}
}


void CmainDlg::OnBnClickedCalib()
{
	if_calib = TRUE;
}


void CmainDlg::OnBnClickedFacial()
{
	if_landmarks = TRUE;
	if_info = FALSE;
}


void CmainDlg::OnBnClickedInfo()
{
	if_landmarks = FALSE;
	if_info = TRUE;
}


void CmainDlg::OnBnClickedDepth()
{
	if_landmarks = FALSE;
	if_info = FALSE;
	if_depth = TRUE;
}


void CmainDlg::OnBnClickedSynth()
{
	if_landmarks = FALSE;
	if_info = FALSE;
	if_depth = FALSE;
	if_synth = TRUE;
}


void CmainDlg::OnBnClickedClose()
{	
	if_landmarks = FALSE;
	if_info = FALSE;
	if_depth = FALSE;
	if_synth = FALSE;
}


void CmainDlg::OnBnClickedDemo()
{
	EndDialog(IDD_MAIN);

	// important to release cameras
	cap_L.release();
	cap_R.release();

	CdemoDlg dlg;
	dlg.DoModal();
}
