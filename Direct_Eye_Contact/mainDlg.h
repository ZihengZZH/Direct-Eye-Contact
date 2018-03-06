#pragma once

#include "resource.h"
#include "faceDepth.h"


/* CLASS OF MFC DIALOG */
// this class is responsible for MFC dialog design

class CmainDlg :public CDialogEx
{
public:
	CmainDlg();
	enum { IDD = IDD_MAIN };
	BOOL OnInitDialog();
	cv::VideoCapture cap_L; // left camera 
	cv::VideoCapture cap_R; // right camera
	cv::Mat cap_mat_L, cap_mat_R;
	cv::Mat mat_depth_standby, mat_synth_standby;
	FaceDepth face;
	BOOL if_landmarks, if_depth, if_synth;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);

protected:
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedOpen();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedDepth();
	afx_msg void OnBnClickedFacial();
	afx_msg void OnBnClickedClose();
	afx_msg void OnBnClickedSynth();
};

