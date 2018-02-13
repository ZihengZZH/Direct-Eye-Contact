#pragma once


class FaceDepth
{

public:
	cv::Mat M1, M2, D1, D2, R, T, R1, R2, P1, P2, Q;
	double focal, baseline, dispar, depth;
	int const max_BINARY_value = 255;
	cv::Mat kernel = cv::Mat(cv::Size(12, 12), CV_8UC1);
	cv::Mat imgLeft_col, imgRight_col;
	dlib::full_object_detection shapes_L, shapes_R;
	std::vector<std::vector<double>> depth_data;

	// face detection and pose estimation parameters 
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor pose_model;
	std::vector<int> facial_point = { 0,17,22,27,31,36 };
	std::vector<std::vector<int>> facial_circle = { { 36,41 },{ 42,47 },{ 48,59 },{ 60,67 } };
	

public:
	FaceDepth() {
		dlib::deserialize("database/shape_predictor_68_face_landmarks.dat") >> pose_model;
	};
	~FaceDepth() {};

public:
	void readPara(void);
	void disparityMap(int ndisparities, int SADWindowSize);
	void facialLandmark(cv::Mat temp, bool left);
	cv::Mat facialLandmarkReal(cv::Mat frame);
	void drawLines(void);
	void saveFile(cv::Mat img_mat);
	void calDepth(void);
};

/*
10 Feb
Use dispariy value first

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
