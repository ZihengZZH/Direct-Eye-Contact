#include "stdafx.h"
#include "mainDlg.h"


CmainDlg::CmainDlg() : CDialogEx(CmainDlg::IDD)
{
}

BOOL CmainDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	cv::namedWindow("view", CV_WINDOW_AUTOSIZE);
	HWND hWnd = (HWND)cvGetWindowHandle("view");
	HWND hParent = ::GetParent(hWnd);
	::SetParent(hWnd, GetDlgItem(IDC_CAM_L)->m_hWnd);
	::ShowWindow(hParent, SW_HIDE);

	return TRUE;
}

void CmainDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CmainDlg, CDialogEx)
	ON_BN_CLICKED(ID_OPEN, &CmainDlg::OnBnClickedOpen)
END_MESSAGE_MAP()

void CmainDlg::OnBnClickedOpen()
{
	std::string pic_path = "./test_image/test.jpg";

	cv::Mat image = cv::imread(pic_path);
	cv::Mat imagedst; // change the size to fit

	CRect rect;
	GetDlgItem(IDC_CAM_L)->GetClientRect(&rect);
	cv::Rect dst(rect.left, rect.top, rect.right, rect.bottom);
	cv::resize(image, imagedst, cv::Size(rect.Width(), rect.Height()));
	cv::imshow("view", imagedst);

}
