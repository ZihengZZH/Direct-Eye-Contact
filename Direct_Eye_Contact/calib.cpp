#include "stdafx.h"
#include "calib.h"


void Calibrate::displayHelp(void)
{
	std::cout << "THIS IS THE STEREO CAMERA CALIBRATION\n"
		<< "-------------------------------------------------------------------------\n"
		<< "To calibrate sterero cameras, twenty pairs of images will be recorded\n"
		<< "Press SPACE to record current stereo images\n"
		<< "And when images are enough, the calibration process will proceed\n"
		<< "IF you have already recorded stereo images\n"
		<< "Press ESC to exit image recording phase.\n"
		<< "-------------------------------------------------------------------------\n"
		<< "Webcameras: Logitech C920 * 2\n"
		<< "-------------------------------------------------------------------------\n\n";
}


bool Calibrate::openCamera(void)
{
	cameraL.open(0);
	cameraR.open(1);
	if (!cameraL.isOpened() || !cameraR.isOpened())
	{
		std::cerr << "FAILED TO OPEN CAMERAS\n";
		return false;
	}
	return true;
}


bool Calibrate::readStringList(const std::string& filename, std::vector<std::string>& l)
{
	l.clear();
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		return false;
	}
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
	{
		return false;
	}
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		l.push_back((std::string)*it);
	}
	return true;
}


void Calibrate::saveTestImage(void)
{

	if (openCamera())
	{
		while (true)
		{
			cameraL >> camera_matL;
			cameraR >> camera_matR;
			imshow("camera L", camera_matL);
			imshow("camera R", camera_matR);
			char key = (char)cv::waitKey(30);
			if (key == ' ')
			{
				saveImage(time);
				time++;
			}
			if (key == 27 || time == times)
			{
				break;
			}
		}
	}
	else {
		std::cerr << "FAILED TO OPEN CAMERAS" << std::endl;
	}
}


void Calibrate::saveImage(int id)
{
	imwrite(imageList[id * 2 + 0], camera_matL);
	imwrite(imageList[id * 2 + 1], camera_matR);
	std::cout << "SAVE IMAGES TO FILES NO _" << id << "_" << std::endl;
}


std::vector<cv::Point3f> Calibrate::Create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float(j*squareSize),
				float(i*squareSize), 0)); // squareSize is vital
		}
	}
	return corners;
}


void Calibrate::stereoCalib(void)
{
	displayHelp();
	readStringList(img_list, imageList);
	if (saveTestImg)
	{
		saveTestImage();
		std::cout << "SUCCESSFULLY WRITTEN IMAGES TO FILES\n";
	}
	if (imageList.size() % 2 != 0)
	{
		std::cerr << "Error: the image list contains odd (non-even) number of elements\n";
	}

	std::vector<std::vector<cv::Point2f>> imagePoints[2];
	std::vector<std::vector<cv::Point3f>> objectPoints;
	cv::Size imageSize;

	int i, j, k, nimages = (int)imageList.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	std::vector<std::string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const std::string filename = imageList[i * 2 + k];
			cv::Mat img = cv::imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == cv::Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			std::vector<cv::Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				cv::Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, cv::Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						cv::Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				std::cout << filename << std::endl;
				cv::Mat cimg, cimg1;
				cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, cv::Size(), sf, sf);
				imshow("corners", cimg1);
				//imwrite(filename, cimg1);
				char c = (char)cv::waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q')
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					30, 0.01));
		}
		if (k == 2)
		{
			goodImageList.push_back(imageList[i * 2]);
			goodImageList.push_back(imageList[i * 2 + 1]);
			j++;
		}
	}
	std::cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		std::cerr << "Error: too little pairs to run the calibration\n";
		return;
	}

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
	}

	std::cout << "Running stereo calibration ...\n";

	cv::Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
	cv::Mat R, T, E, F;
	// R - output rotation matrix between the 1st and the 2nd camera coordinate systems
	// T - output translation vector between the coordinate systems of the cameras
	// E - output essential matrix
	// F - output fundamental matrix

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO +
		cv::CALIB_ZERO_TANGENT_DIST +
		cv::CALIB_USE_INTRINSIC_GUESS +
		cv::CALIB_SAME_FOCAL_LENGTH +
		cv::CALIB_RATIONAL_MODEL +
		cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
	std::cout << "done with RMS error=" << rms << std::endl;

	// CALIBRATION QUALITY CHECK
	double err = 0;
	int npoints = 0;
	std::vector<cv::Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		cv::Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = cv::Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	std::cout << "average epipolar err = " << err / npoints << std::endl;

	// Save intrinsic parameters
	cv::FileStorage fs(cam_intri, cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		std::cerr << "Error: can not save the intrinsic parameters\n";

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	// R1 - output 3*3 rectification transform (rotation matrix) for the first camera
	// R2 - output 3*3 rectification transform (rotation matrix) for the second camera
	// P1 - output 3*4 projection matrix in the new (rectified) coordinate systems for the first camera
	// P2 - output 3*4 projection matrix in the new (rectified) coordinate systems for the second camera
	// Q - output 4*4 disparity-to-depth mapping matrix

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open(cam_extri, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		std::cerr << "Error: can not save the extrinsic parameters\n";

	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	cv::Mat rmap[2][2];
	if (useCalibrated)
	{
	}
	else {
		std::vector<cv::Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
		cv::Mat H1, H2;
		stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			cv::Mat img = cv::imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);
			cv::cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
			cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
			cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
			if (useCalibrated)
			{
				cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)cv::waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
	cv::destroyAllWindows();
}


void Calibrate::readParameter(void)
{
	cv::FileStorage fs;
	fs.open(cam_intri, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> M1;
		fs["M2"] >> M2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
		fs.release();
	}
	else
		std::cerr << "ERROR READING PARA\n";

	fs.open(cam_extri, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else
		std::cerr << "ERROR READING PARA\n";
}


void Calibrate::rectifyImage(std::string faceL, std::string faceR)
{

	readParameter();
	cv::Size imageSize;
	cv::Mat rmap[2][2];
	cv::Mat face0 = cv::imread(faceL, 0);
	cv::Mat face1 = cv::imread(faceR, 0);
	if (face0.size() == face1.size())
		imageSize = face0.size();
	else
		return;

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat canvas;
	double sf;
	int w, h;

	sf = 600. / MAX(imageSize.width, imageSize.height);

	w = cvRound(imageSize.width*sf);
	h = cvRound(imageSize.height*sf);
	canvas.create(h, w * 2, CV_8UC3);


	for (int k = 0; k < 2; k++)
	{
		cv::Mat img = (k == 0) ? face0 : face1;
		cv::Mat rimg, cimg;
		cv::remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);
		cv::cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
		if (k == 0)
		{
			//cv::imshow("rimgL", rimg);
			cv::imwrite(face_outL, rimg);
		}
		else
		{
			//cv::imshow("rimgR", rimg);
			cv::imwrite(face_outR, rimg);
		}
		cv::Mat canvasPart = canvas(cv::Rect(w*k, 0, w, h));
		cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
	}

	for (int j = 0; j < canvas.rows; j += 16)
		line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);

	cv::namedWindow("rectified", CV_WINDOW_AUTOSIZE);
	cv::imshow("rectified", canvas);
	char c = (char)cv::waitKey();
	if (c == 27 || c == 'q' || c == 'Q')
		return;

}
