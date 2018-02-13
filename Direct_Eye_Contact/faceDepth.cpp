#include "stdafx.h"
#include "faceDepth.h"


void FaceDepth::readPara(void)
{
	cv::FileStorage fs;
	fs.open("calib_xml/intrinsics.yml", cv::FileStorage::READ);
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

	/*
	camera matrix M1 M2
	[ fx  0  cx ]
	[ 0  fy  cy ]
	[ 0   0   1 ]
	fx, fy: focal length
	cx, cy: principal point coordinates
	*/

	fs.open("calib_xml/extrinsics.yml", cv::FileStorage::READ);
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

	Q.convertTo(Q, CV_64F);

	focal = (M1.ptr<double>(0)[0] + M1.ptr<double>(1)[1]
		+ M2.ptr<double>(0)[0] + M2.ptr<double>(1)[1]) / 4;
	baseline = 1 / Q.ptr<double>(3)[2];

}


// Tradition method, but not suitable for the project
void FaceDepth::disparityMap(int ndisparities, int SADWindowSize)
{
	cv::Mat imgLeft, imgRight;
	double minVal, maxVal;
	cv::Size size(640, 480);

	cv::cvtColor(imgLeft_col, imgLeft, CV_BGR2GRAY);
	cv::cvtColor(imgRight_col, imgRight, CV_BGR2GRAY);

	cv::resize(imgLeft, imgLeft, size);
	cv::resize(imgRight, imgRight, size);

	// These parameters should be adjusted to specific cameras
	//int ndisparities = 16, SADWindowSize = 15;
	int mindisparity = 0;
	//Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
	cv::Ptr<cv::StereoSGBM> sbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize, 216, 864, 1, 63, 0, 100, 32);

	/*
	There is a huge difference between BM algorithm and SGBM algorithm
	The implementation of BM is quick with satisfactory resolution
	But the implementation of SGBM has finer resolution at the expense of low speed
	*/

	cv::Mat imgDisparity16S = cv::Mat(imgLeft.rows, imgLeft.cols, CV_16S);
	cv::Mat imgDisparity8U = cv::Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);

	// Compute disparity map for the specified stereo pair
	sbm->compute(imgLeft, imgRight, imgDisparity16S);
	minMaxLoc(imgDisparity16S, &minVal, &maxVal);
	std::cout << "Min disp " << minVal << " Max disp " << maxVal << std::endl;
	imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255 / (maxVal - minVal));
	// 255 / (maxVal - minVal)

	cv::Mat disp, disp8, points;
	sbm->compute(imgLeft, imgRight, disp);
	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	cv::imshow("Left view", imgLeft_col);
	cv::imshow("Right view", imgRight_col);
	cv::imshow("Disparity map", imgDisparity8U);
	cv::waitKey();

}


void FaceDepth::facialLandmark(cv::Mat temp, bool left)
{

	dlib::cv_image<dlib::bgr_pixel> cimg(temp);
	// Detect faces
	std::vector<dlib::rectangle> faces = detector(cimg);
	// Find the pose of each face. 
	std::vector<dlib::full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(pose_model(cimg, faces[i]));

	if (!shapes.empty())
	{
		/*for (int i = 0; i < 68; i++)
		{
			circle(temp, cvPoint(shapes[0].part(i).x(), shapes[0].part(i).y()),
				3, cv::Scalar(0, 0, 255), -1);
			putText(temp, std::to_string(i), cvPoint(shapes[0].part(i).x(),
				shapes[0].part(i).y()), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 1, 4);
			//  shapes[0].part(i).x();
			// std::cout << "No" << i << " x " << shapes[0].part(i).x() << "\ty " << shapes[0].part(i).y() << std::endl;
		}*/
		for (int i = 0; i < 68; i++)
		{
			circle(temp, cvPoint(shapes[0].part(i).x(), shapes[0].part(i).y()),
				3, cv::Scalar(0, 255, 0), -1);
		}

		for (int iter = 0; iter != facial_point.size() - 1; iter++)
		{
			for (int i = facial_point[iter]; i != facial_point[iter + 1] - 1; i++)
			{
				cv::line(temp, cv::Point(shapes[0].part(i).x(), shapes[0].part(i).y()),
					cv::Point(shapes[0].part(i + 1).x(), shapes[0].part(i + 1).y()), cv::Scalar(0, 255, 0));
			}
		}
		for (auto point : facial_circle)
		{
			for (int i = point[0]; i != point[1]; i++)
			{
				cv::line(temp, cv::Point(shapes[0].part(i).x(), shapes[0].part(i).y()),
					cv::Point(shapes[0].part(i + 1).x(), shapes[0].part(i + 1).y()), cv::Scalar(0, 255, 0));
			}
			cv::line(temp, cv::Point(shapes[0].part(point[1]).x(), shapes[0].part(point[1]).y()),
				cv::Point(shapes[0].part(point[0]).x(), shapes[0].part(point[0]).y()), cv::Scalar(0, 255, 0));
		}

	}

	if (left)
		shapes_L = shapes[0];
	else
		shapes_R = shapes[0];

	//Display it all on the screen  
	/*cv::imshow("Dlib facial landmarks", temp);
	char c = (char)cv::waitKey(1);
	if (c == 27)
	{
		return;
	}
	cv::destroyAllWindows();*/

}

cv::Mat FaceDepth::facialLandmarkReal(cv::Mat frame)
{
	cv::Mat frame_facial;
	frame.copyTo(frame_facial);

	dlib::cv_image<dlib::bgr_pixel> cimg(frame_facial);
	// Detect faces
	std::vector<dlib::rectangle> faces = detector(cimg);
	// Find the pose of each face. 
	std::vector<dlib::full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(pose_model(cimg, faces[i]));

	if (!shapes.empty())
	{
		/*for (int i = 0; i < 68; i++)
		{
		circle(temp, cvPoint(shapes[0].part(i).x(), shapes[0].part(i).y()),
		3, cv::Scalar(0, 0, 255), -1);
		putText(temp, std::to_string(i), cvPoint(shapes[0].part(i).x(),
		shapes[0].part(i).y()), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 1, 4);
		//  shapes[0].part(i).x();
		// std::cout << "No" << i << " x " << shapes[0].part(i).x() << "\ty " << shapes[0].part(i).y() << std::endl;
		}*/
		for (int i = 0; i < 68; i++)
		{
			circle(frame_facial, cvPoint(shapes[0].part(i).x(), shapes[0].part(i).y()),
				3, cv::Scalar(0, 255, 0), -1);
		}

		for (int iter = 0; iter != facial_point.size() - 1; iter++)
		{
			for (int i = facial_point[iter]; i != facial_point[iter + 1] - 1; i++)
			{
				cv::line(frame_facial, cv::Point(shapes[0].part(i).x(), shapes[0].part(i).y()),
					cv::Point(shapes[0].part(i + 1).x(), shapes[0].part(i + 1).y()), cv::Scalar(0, 255, 0));
			}
		}
		for (auto point : facial_circle)
		{
			for (int i = point[0]; i != point[1]; i++)
			{
				cv::line(frame_facial, cv::Point(shapes[0].part(i).x(), shapes[0].part(i).y()),
					cv::Point(shapes[0].part(i + 1).x(), shapes[0].part(i + 1).y()), cv::Scalar(0, 255, 0));
			}
			cv::line(frame_facial, cv::Point(shapes[0].part(point[1]).x(), shapes[0].part(point[1]).y()),
				cv::Point(shapes[0].part(point[0]).x(), shapes[0].part(point[0]).y()), cv::Scalar(0, 255, 0));
		}

	}

	return frame_facial;
}


void FaceDepth::drawLines(void)
{
	cv::Mat img1 = imgLeft_col, img2 = imgRight_col;

	int rows = std::max(img1.rows, img2.rows);
	int cols = img1.cols + img2.cols;

	// Create a black image
	cv::Mat3b img_res(rows, cols, cv::Vec3b(0, 0, 0));

	// Copy images in correct position
	img1.copyTo(img_res(cv::Rect(0, 0, img1.cols, img1.rows)));
	img2.copyTo(img_res(cv::Rect(img1.cols, 0, img2.cols, img2.rows)));

	//-- Draw lines between the corners
#pragma omp parallel
#pragma omp for
	for (int i = 0; i < 68; i++)
	{
		cv::line(img_res, cv::Point(shapes_L.part(i).x(), shapes_L.part(i).y()),
			cv::Point(shapes_R.part(i).x() + img1.cols, shapes_R.part(i).y()), cv::Scalar(0, 255, 0));
		/*std::cout << "No" << i << " x " << shapes_L.part(i).x() << " " << shapes_R.part(i).x()
		<< "\ty " << shapes_L.part(i).y() << " " << shapes_R.part(i).y() << std::endl;*/
	}

	cv::imshow("matches", img_res);
	char key = (char)cv::waitKey();
	if (key == 27)
		return;
	cv::destroyAllWindows();
}


void FaceDepth::saveFile(cv::Mat img_mat)
{
	cv::FileStorage file;
	file.open("depth/depth_info.yml", cv::FileStorage::WRITE);
	if (!file.isOpened()) {
		std::cerr << "Failed to open" << std::endl;
		return;
	}
	file << "depth info" << img_mat;
	file.release();
}


void FaceDepth::calDepth(void)
{
	imgLeft_col = cv::imread("test_image/face_rec0_L.jpg", 1); //face_rec0.jpg
	imgRight_col = cv::imread("test_image/face_rec1_R.jpg", 1); //face_rec1.jpg

	readPara();
	focal = 0.00367; // unit mt
	std::cout << "focal length " << focal << std::endl;
	std::cout << "baseline " << baseline << std::endl;
	facialLandmark(imgLeft_col, true);
	facialLandmark(imgRight_col, false);

	std::cout << "LEFT & RIGHT\n";
	for (int i = 0; i < 68; i++)
	{
		dispar = double(shapes_L.part(i).x() - shapes_R.part(i).x());
		depth = baseline * focal / dispar; // depth computation
		std::cout << "No" << i << " x " << shapes_L.part(i).x() << " " << shapes_R.part(i).x()
			<< "\ty " << shapes_L.part(i).y() << " " << shapes_R.part(i).y() << "\t disp " << dispar
			<< "\t depth " << depth << std::endl;
		//std::vector<double> depth_d = { (double)i, (double)shapes_L.part(i).x(), (double)shapes_L.part(i).y(), depth };
		//depth_data.push_back(depth_d);
	}

	drawLines();
}
