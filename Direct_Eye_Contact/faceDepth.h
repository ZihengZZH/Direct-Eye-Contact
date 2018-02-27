#pragma once


class FaceDepth
{

public:
	cv::Mat M1, M2, D1, D2, R, T, R1, R2, P1, P2, Q;
	double focal, baseline, dispar, depth;

	cv::Mat imgLeft_col, imgRight_col; // undistort frame from web-cam
	dlib::full_object_detection shapes_L, shapes_R;
	
	std::vector<double> depth_data; // depth data of landmarks
	std::vector<double> original_pos, virtual_pos;
	std::vector<std::pair<int, double>> depth_data_index = std::vector<std::pair<int, double>>(68);
	//std::vector<std::pair<int, double>> level_1, level_2, level_3;

private:
	// face detection and pose estimation parameters 
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor pose_model;
	std::vector<int> facial_point = { 0,17,22,27,31,36 };
	std::vector<std::vector<int>> facial_circle = { { 36,41 },{ 42,47 },{ 48,59 },{ 60,67 } };

	struct CmpByValue
	{
		bool operator()
			(const std::pair<int, double> & lhs, const std::pair<int, double> & rhs)
		{
			return lhs.second > rhs.second;
		}
	};

public:
	FaceDepth();
	~FaceDepth() {};

public:
	cv::Rect dlib2opencv(dlib::rectangle r);
	void readPara(void);
	void disparityMap(int ndisparities, int SADWindowSize);

	bool facialLandmark(bool left);
	// store the shape of landmarks
	cv::Mat facialLandmarkReal(bool left);
	// return mat in real-time
	cv::Mat drawLines(void);

	void levelDepth(cv::Mat& img);
	void saveFile(cv::Mat img_mat);
	void calDepth(void);
	void calTranslation(bool vir_cam); // not suitable yet
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

	19 Feb
	Z = fB/d
	where
	Z = distance along the camera Z axis
	f = focal length (in pixels)
	B = baseline (in metres)
	d = disparity (in pixels)
	SO no need to worry the sensor size

*/