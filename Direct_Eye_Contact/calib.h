#pragma once

class Calibrate
{

private:
	cv::VideoCapture cameraL;
	cv::VideoCapture cameraR;
	cv::Mat camera_matL;
	cv::Mat camera_matR;
	std::vector<std::string> imageList;
	const int times = 20;
	int time = 0;
	std::string img_list = "calib_xml/imageList.xml";
	std::string cam_intri = "calib_xml/intrinsics.yml";
	std::string cam_extri = "calib_xml/extrinsics.yml";
	std::string face_outL = "test_image/face_rec0_L.jpg";
	std::string face_outR = "test_image/face_rec1_R.jpg";

private:
	float squareSize = 0.025f; // The square size is 2.5cm
	bool displayCorners = true;
	bool useCalibrated = true;
	bool showRectified = true;
	bool saveTestImg = true;
	cv::Size boardSize = cv::Size(6, 9); // The chessboard is 7*10
	const int maxScale = 2;

	/*
	squareSize is important for the calibration as it determines the relationship
	the square size is the side length of each square on the chessboard
	in the experiment, it is 25mm, which is 0.025f
	And the baseline is measured 10cm / 100mm / 0.1m

	focal length is another vital aspect
	in the camera matrix, fx and fy should have the same value, which is measured in pixels
	C920 sensor size 5.14mm * 3.50mm
	optical resolution 3MP	2048*1536
	Fx = fx * W / w
	focal length 3.67mm
	*/

public:
	// calibration parameters
	cv::Mat M1, M2, D1, D2, R, T, R1, R2, P1, P2, Q;


public:
	Calibrate() {};
	~Calibrate() {};

	void displayHelp(void);
	bool openCamera(void);
	bool readStringList(const std::string& filename, std::vector<std::string>& l);
	void saveTestImage(void);
	void saveImage(int id);
	std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize);
	void stereoCalib(void);
	void readPara(void);
	void rectifyImage(std::string faceL, std::string faceR);


};

