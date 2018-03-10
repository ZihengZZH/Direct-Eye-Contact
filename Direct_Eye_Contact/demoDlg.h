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

	Calibrate calib;
	FaceDepth face;
	BOOL if_synth;
	CFont m_font;
	HICON m_hIcon;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);

protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedStart();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

