// Direct_Eye_Contact.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "calib.h"

int main()
{
	Calibrate start;
	//start.stereoCalib();
	
	/*cv::Mat faceL, faceR;

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
	}*/

	start.rectifyImage("test_image/faceL.jpg", "test_image/faceR.jpg");
	
    return 0;
}

