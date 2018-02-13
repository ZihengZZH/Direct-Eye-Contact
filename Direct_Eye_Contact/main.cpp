// Direct_Eye_Contact.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "calib.h"
#include "faceDepth.h"


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
	//face.calDepth();

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

		camera_matL = face.facialLandmarkReal(camera_matL);
		//camera_matR = face.facialLandmarkReal(camera_matR);
		cv::namedWindow("Left camera view", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Right camera view", CV_WINDOW_AUTOSIZE);
		cv::imshow("Left camera view", camera_matL);
		cv::imshow("Right camera view", camera_matR);

		char c = (char)cv::waitKey(1);
		if (c == 27)
		{
			break;
		}
	}


}