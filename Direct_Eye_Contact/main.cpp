// Direct_Eye_Contact.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "resource.h"
#include "calib.h"
#include "faceDepth.h"



/*
// Functions definition
void calibrateStereo();
void faceDepth();


int main()
{
	//calibrateStereo();
	faceDepth();

	return 0;
}


void calibrateStereo()
{
	Calibrate calib;
	//calib.stereoCalib();

	cv::Mat faceL, faceR;

	// RECORD THE FACES
	cv::VideoCapture cameraL(0);
	cv::VideoCapture cameraR(1);
	if (!cameraL.isOpened() || !cameraR.isOpened())
	{
		std::cerr << "FAILED TO OPEN CAMERAS\n";
	}
	cv::Mat camera_matL, camera_matR;
	while (true)
	{
		cameraL >> camera_matL;
		cameraR >> camera_matR;

		cv::namedWindow("Left camera view", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Right camera view", CV_WINDOW_AUTOSIZE);
		cv::imshow("Left camera view", camera_matL);
		cv::imshow("Right camera view", camera_matR);

		char c = (char)cv::waitKey(1);
		if (c == ' ')
		{
			cv::imwrite("test_image/faceL.jpg", camera_matL);
			cv::imwrite("test_image/faceR.jpg", camera_matR);
		}
		if (c == 27)
		{
			break;
		}
	}

	calib.rectifyImage("test_image/faceL.jpg", "test_image/faceR.jpg");

}


void faceDepth()
{
	FaceDepth face;
	face.readPara();

	cv::VideoCapture cameraL(0);
	cv::VideoCapture cameraR(1);
	if (!cameraL.isOpened() || !cameraR.isOpened())
	{
		std::cerr << "FAILED TO OPEN CAMERAS\n";
	}

	cv::Mat camera_matL, camera_matR;
	cv::Mat matL, matR, mat;
	while (true)
	{
		cameraL >> camera_matL;
		cameraR >> camera_matR;

		// REAL TIME RECTIFICATION OF VIDEO CAPTURE
		cv::undistort(camera_matL, face.imgLeft_col, face.M1, face.D1);
		cv::undistort(camera_matR, face.imgRight_col, face.M2, face.D2);

		face.imgLeft_col.copyTo(matL);


		face.imgRight_col.copyTo(matR);

		//matL = face.facialLandmarkVis(true);
		//matR = face.facialLandmarkVis(false);

		// MEMORY BUG EVERYWHERE
		if (face.facialLandmark(true) && face.facialLandmark(false))
		{
			face.calDepth();
			face.levelDepthVis(matL);
			//face.levelDepth(matL); // store depth info
		}


		//face.drawLevel(matR);

		cv::namedWindow("Left camera view", CV_WINDOW_AUTOSIZE);
		//cv::namedWindow("Right camera view", CV_WINDOW_AUTOSIZE);
		cv::imshow("Left camera view", matL);
		//cv::imshow("Right camera view", matR);

		char c = (char)cv::waitKey(1);
		if (c == 27)
			break;
	}


}
*/

/* CONSOLE APPLICATION TO MFC DIALOG APPLICATION */
// Property -> General -> Use of MFC (Windows Libraries -> Shared DLL)
// Property -> Linker -> System -> SubSystem (Console -> Windows)
// Property -> C/C++ -> Advanced -> Show includes (IN CASE HEADERS ERRPR)

class CMAINDlg :public CDialogEx
{
public:
	CMAINDlg();
	enum { IDD = IDD_MAIN };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

protected:
	DECLARE_MESSAGE_MAP()

};

CMAINDlg::CMAINDlg() : CDialogEx(CMAINDlg::IDD)
{
}

void CMAINDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CMAINDlg, CDialogEx)
END_MESSAGE_MAP()


class CMAINApp : public CWinApp
{
public:
	BOOL InitInstance();
};

BOOL CMAINApp::InitInstance()
{
	CMAINDlg dlg;
	m_pMainWnd = &dlg;
	dlg.DoModal();

	return TRUE;
}

CMAINApp theApp;