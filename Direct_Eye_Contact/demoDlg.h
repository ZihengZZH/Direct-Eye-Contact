#pragma once

#include "resource.h"
#include "faceDepth.h"
#include "calib.h"
#include "mainDlg.h"
#include "afxwin.h"


/* CLASS OF DEMO DIALOG */
// this class is responsible for demo dialog of demonstration

class CdemoDlg :public CDialogEx
{
public:
	CdemoDlg();
	enum { IDD = IDD_DEMO };
	BOOL OnInitDialog();

private:
	cv::VideoCapture cap_L; // left camera
	cv::VideoCapture cap_R; // right camera
	cv::Mat cap_mat_L, cap_mat_R;
	cv::Mat cap_mat_L_calib, cap_mat_R_calib;

    // variables for rectification
    cv::Mat rmap[2][2];
    cv::Size imageSize = cv::Size(640, 480);

	Calibrate calib;
	FaceDepth face;
	BOOL if_synth;
	CFont m_font;
	HICON m_hIcon;
    DWORD start_time, end_time;
    UINT frame_second, count = 0;
    std::string fps;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);

protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedStart();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

