#pragma once

#include "resource.h"
#include "faceDepth.h"
#include "calib.h"
#include "demoDlg.h"
#include "afxwin.h"


/* CLASS OF MFC DIALOG */
// this class is responsible for MFC dialog design

class CmainDlg :public CDialogEx
{
public:
	CmainDlg();
	enum { IDD = IDD_MAIN };
	BOOL OnInitDialog();
	void EndDialog(int nResult);

private:
	cv::VideoCapture cap_L; // left camera 
	cv::VideoCapture cap_R; // right camera
	cv::Mat cap_mat_L, cap_mat_R;
	cv::Mat cap_mat_L_calib, cap_mat_R_calib;
	cv::Mat mat_depth_standby, mat_synth_standby;
	
	Calibrate calib;
	FaceDepth face;
	BOOL if_record, if_calib, if_landmarks, if_info, if_depth, if_synth;
	BOOL already_calib;
	CEdit* pBoxOne;
	CString m_str;
	CImage m_logo;
	CFont m_font, m_font_small;
	HICON m_hIcon;

	enum
	{
		USE_LEVEL = 0,
		USE_VORONOI
	};
	int depth_method = 1;


protected:
	virtual void DoDataExchange(CDataExchange* pDX);

protected:
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedOpen();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedRecord();
	afx_msg void OnBnClickedCalib();
	afx_msg void OnBnClickedDepth();
	afx_msg void OnBnClickedFacial();
	afx_msg void OnBnClickedInfo();
	afx_msg void OnBnClickedClose();
	afx_msg void OnBnClickedSynth();
	afx_msg void OnBnClickedDemo();

	CStatic m_Logo;
	CStatic m_icon;
	CStatic m_info;

};

