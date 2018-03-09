#include "stdafx.h"
#include "faceDepth.h"


static void average_vec(std::vector<std::pair<int, double> >& temp, double& avg)
{
	double sum = 0;
	for (auto t : temp)
		sum += t.second;
	avg = sum / temp.size();
}


static void median_vec(std::vector<std::pair<int, double>>&temp, double& median)
{
	if (temp.size() % 2 == 0)
	{
		median = (temp[temp.size() / 2 - 1].second + temp[temp.size() / 2].second) / 2;
	}
	else
	{
		median = temp[temp.size() / 2].second;
	}
}


FaceDepth::FaceDepth()
{
	dlib::deserialize("database/shape_predictor_68_face_landmarks.dat") >> pose_model;
	depth_data.clear();
	original_pos.clear();
	virtual_pos.clear();
	level_method = USE_MEDIAN;
}


cv::Rect FaceDepth::dlib2opencv(dlib::rectangle r)
{
	return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
}


bool FaceDepth::readParameter(void)
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
		return false;

	/*
		camera matrix M1 M2
		[ fx  0  cx ]
		[ 0  fy  cy ]
		[ 0   0   1 ]
		fx, fy: focal length
		cx, cy: principal point coordinates
		resolution: 640 * 480 pixel
		sensor-size: 5.14 * 3.5 mm
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
		return false;

	Q.convertTo(Q, CV_64F);
	R.convertTo(R, CV_64F);
	T.convertTo(T, CV_64F);
	M1.convertTo(M1, CV_64F);
	M2.convertTo(M2, CV_64F);

	focal = (M1.ptr<double>(0)[0] + M1.ptr<double>(1)[1]
		+ M2.ptr<double>(0)[0] + M2.ptr<double>(1)[1]) / 4; // in pixel
	baseline = 1 / Q.ptr<double>(3)[2]; // in meters
	
	return true;
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


// store the shape of landmarks to the shape variable
bool FaceDepth::facialLandmark(bool left)
{
	cv::Mat frame_facial;
	if (left)
		imgLeft_col.copyTo(frame_facial);
	else
		imgRight_col.copyTo(frame_facial);

	dlib::cv_image<dlib::bgr_pixel> cimg(frame_facial);
	// Detect faces
	std::vector<dlib::rectangle> faces = detector(cimg);
	// Find the pose of each face. 
	std::vector<dlib::full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(pose_model(cimg, faces[i]));

	if (!shapes.empty())
	{
		if (left)
		{
			shapes_L = shapes[0];

			int border_h = shapes[0].part(36).x() - shapes[0].part(0).x();
			int border_v = shapes[0].part(38).y() - shapes[0].part(19).y();
			cv::Rect face = dlib2opencv(faces[0]);
			face.x -= border_h;
			face.y -= border_v * 2;
			face.width += border_h * 2;
			face.height += border_v * 2;

			face_rect = face;
			borders[0] = cv::Point2i(face.x, face.y);
			borders[1] = cv::Point2i(face.x, face.y + face.height);
			borders[2] = cv::Point2i(face.x + face.width, face.y + face.height);
			borders[3] = cv::Point2i(face.x + face.width, face.y);
		}
		else
			shapes_R = shapes[0];

		return true;
	}
	else
		return false;

}


// return the frame with drawing to display
cv::Mat FaceDepth::facialLandmarkVis(bool left)
{
	cv::Mat frame_facial;
	if (left)
		imgLeft_col.copyTo(frame_facial);
	else
		imgRight_col.copyTo(frame_facial);

	dlib::cv_image<dlib::bgr_pixel> cimg(frame_facial);
	// Detect faces
	std::vector<dlib::rectangle> faces = detector(cimg);
	// Find the pose of each face. 
	std::vector<dlib::full_object_detection> shapes;
	for (unsigned long i = 0; i < faces.size(); ++i)
		shapes.push_back(pose_model(cimg, faces[i]));


	if (!shapes.empty())
	{
		int border_h = shapes[0].part(36).x() - shapes[0].part(0).x();
		int border_v = shapes[0].part(38).y() - shapes[0].part(19).y();

		cv::Rect face = dlib2opencv(faces[0]);
		face.x -= border_h / 2;
		face.y -= border_v * 2;
		face.width += border_h;
		face.height += border_v * 3;
		cv::rectangle(frame_facial, face, cv::Scalar(0, 255, 0), 0.7);

		for (int i = 0; i < 68; i++)
		{
			circle(frame_facial, cvPoint(shapes[0].part(i).x(), shapes[0].part(i).y()),
				2, cv::Scalar(0, 255, 0), -1);
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


cv::Mat FaceDepth::drawLines(void)
{
	// RUN facialLandmark FIRST

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
	}
	return img_res;
}


void FaceDepth::levelDepth(cv::Mat & img)
{
	std::vector<std::pair<int, double>> level_1, level_2, level_3; 
	// 1 is farthest(largest) and 3 is closest(smallest)
	// the coordinates may be easier to be accessed in vector
	std::vector<cv::Point2i> points_L, points_R, convexHull;
	std::vector<cv::Point2i> points_1, points_2, points_3;
	std::vector<cv::Point2i> contour_1, contour_2, contour_3;
	double average_1, average_2, average_3;
	double median_1, median_2, median_3;
	double level_val_0, level_val_1, level_val_2, level_val_3;
	cv::Mat imgDepth64F = cv::Mat(img.rows, img.cols, CV_64F);

	for (int k = 0; k < 68; k++)
	{
		points_L.push_back(cv::Point2i(shapes_L.part(k).x(), shapes_L.part(k).y()));
		points_R.push_back(cv::Point2i(shapes_R.part(k).x(), shapes_R.part(k).y()));
	}

	std::sort(depth_data_index.begin(), depth_data_index.end(), CmpByValue());

	int number = depth_data_index.size() / 3;

	// the vectors are already sorted
	for (int i = 0; i < depth_data_index.size(); i++)
	{
		if (i > number * 2)
			level_3.push_back(depth_data_index[i]);
		else if (i > number * 1)
			level_2.push_back(depth_data_index[i]);
		else
			level_1.push_back(depth_data_index[i]);
	}
	
	for (auto one : level_1)
		points_1.push_back(points_L[one.first]);
	for (auto two : level_2)
		points_2.push_back(points_L[two.first]);
	for (auto three : level_3)
		points_3.push_back(points_L[three.first]);

	cv::convexHull(cv::Mat(points_1), convexHull, false);
	cv::approxPolyDP(cv::Mat(convexHull), contour_1, 0.001, true);
	cv::convexHull(cv::Mat(points_2), convexHull, false);
	cv::approxPolyDP(cv::Mat(convexHull), contour_2, 0.001, true);
	cv::convexHull(cv::Mat(points_3), convexHull, false);
	cv::approxPolyDP(cv::Mat(convexHull), contour_3, 0.001, true);

	// TWO METHODS TO SET THE REPRESENTATIVE VALUE FOR EACH LEVEL
	if (level_method == USE_AVERAGE)
	{
		average_vec(level_1, average_1);
		average_vec(level_2, average_2);
		average_vec(level_3, average_3);
		level_val_1 = average_1;
		level_val_2 = average_2;
		level_val_3 = average_3;
	}
	if (level_method == USE_MEDIAN)
	{
		median_vec(level_1, median_1);
		median_vec(level_2, median_2);
		median_vec(level_3, median_3);
		level_val_1 = median_1;
		level_val_2 = median_2;
		level_val_3 = median_3;
	}

	level_val_0 = 1000 * level_val_1; // DEFAULT background with infinite distance
	cv::rectangle(imgDepth64F, face_rect, cv::Scalar(level_val_0), CV_FILLED);
	cv::fillConvexPoly(imgDepth64F, contour_1, cv::Scalar(level_val_1));
	cv::fillConvexPoly(imgDepth64F, contour_2, cv::Scalar(level_val_2));
	cv::fillConvexPoly(imgDepth64F, contour_3, cv::Scalar(level_val_3));
	imgDepth64F = imgDepth64F(face_rect);
	//saveFile(imgDepth64F);
}


void FaceDepth::levelDepthVis(cv::Mat& img, bool if_info)
{
	if (if_info)
	{
		float point_8, point_30, point_57;
		point_8 = baseline * focal / (shapes_L.part(8).x() - shapes_R.part(8).x());
		point_30 = baseline * focal / (shapes_L.part(30).x() - shapes_R.part(30).x());
		point_57 = baseline * focal / (shapes_L.part(57).x() - shapes_R.part(57).x());
		circle(img, cvPoint(shapes_L.part(8).x(), shapes_L.part(8).y()), 3, cv::Scalar(0, 255, 0), -1);
		circle(img, cvPoint(shapes_L.part(30).x(), shapes_L.part(30).y()), 3, cv::Scalar(0, 255, 0), -1);
		circle(img, cvPoint(shapes_L.part(57).x(), shapes_L.part(57).y()), 3, cv::Scalar(0, 255, 0), -1);

		putText(img, std::to_string(point_8), cvPoint(shapes_L.part(8).x(),
			shapes_L.part(8).y()), 1, 1, cv::Scalar(255, 0, 0), 1, 4);
		putText(img, std::to_string(point_30), cvPoint(shapes_L.part(30).x(),
			shapes_L.part(30).y()), 1, 1, cv::Scalar(255, 0, 0), 1, 4);
		putText(img, std::to_string(point_57), cvPoint(shapes_L.part(57).x(),
			shapes_L.part(57).y()), 1, 1, cv::Scalar(255, 0, 0), 1, 4);
	}
	else
	{
		std::vector<std::pair<int, double>> level_1, level_2, level_3; // 3 closest
		// the coordinates may be easier to be accessed in vector
		// Point2i (integer) vital to draw the convex polygon
		std::vector<cv::Point2i> points_L, points_R, convexHull;
		std::vector<cv::Point2i> points_1, points_2, points_3;
		std::vector<cv::Point2i> contour_1, contour_2, contour_3;
		bool show_points = false;

		for (int k = 0; k < 68; k++)
		{
			points_L.push_back(cv::Point2i(shapes_L.part(k).x(), shapes_L.part(k).y()));
			points_R.push_back(cv::Point2i(shapes_R.part(k).x(), shapes_R.part(k).y()));
		}

		std::sort(depth_data_index.begin(), depth_data_index.end(), CmpByValue());

		int number = depth_data_index.size() / 3;

		for (int i = 0; i < depth_data_index.size(); i++)
		{
			if (i > number * 2)
				level_3.push_back(depth_data_index[i]);
			else if (i > number * 1)
				level_2.push_back(depth_data_index[i]);
			else
				level_1.push_back(depth_data_index[i]);
		}

		// Scalar BGR (Blue Green Red)
		for (auto one : level_1)
		{
			if (show_points)
				cv::circle(img, points_L[one.first], 2, cv::Scalar(255, 0, 0)); // BLUE
			points_1.push_back(points_L[one.first]);
		}
		for (auto two : level_2)
		{
			if (show_points)
				cv::circle(img, points_L[two.first], 2, cv::Scalar(0, 255, 0)); // GREEN
			points_2.push_back(points_L[two.first]);
		}
		for (auto three : level_3)
		{
			if (show_points)
				cv::circle(img, points_L[three.first], 2, cv::Scalar(0, 0, 255)); // RED
			points_3.push_back(points_L[three.first]);
		}

		cv::convexHull(cv::Mat(points_1), convexHull, false);
		cv::approxPolyDP(cv::Mat(convexHull), contour_1, 0.001, true);
		cv::convexHull(cv::Mat(points_2), convexHull, false);
		cv::approxPolyDP(cv::Mat(convexHull), contour_2, 0.001, true);
		cv::convexHull(cv::Mat(points_3), convexHull, false);
		cv::approxPolyDP(cv::Mat(convexHull), contour_3, 0.001, true);

		cv::fillConvexPoly(img, contour_1, cv::Scalar(100, 100, 100));
		cv::fillConvexPoly(img, contour_2, cv::Scalar(150, 150, 150));
		cv::fillConvexPoly(img, contour_3, cv::Scalar(200, 200, 200));
	}
	

	// NO NEED TO DEALLOCATE THE VECTORS
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
	// READ PARAMETERS FIRST
	//readParameter();
	//std::cout << "focal length " << focal << std::endl;
	//std::cout << "baseline " << baseline << std::endl;

	for (int i = 0; i < 68; i++)
	{
		dispar = double(shapes_L.part(i).x() - shapes_R.part(i).x());
		depth = baseline * focal / dispar; // depth computation

		/*std::cout << "No" << i << " x " << shapes_L.part(i).x() << " " << shapes_R.part(i).x()
			<< "\ty " << shapes_L.part(i).y() << " " << shapes_R.part(i).y() << "\t disp " << dispar
			<< "\t depth " << depth << std::endl;*/
			//depth_data.push_back(depth);
		depth_data_index[i] = std::make_pair(i, depth);
	}

	//drawLines();
}


void FaceDepth::calTranslation(bool vir_cam)
{
	// READ PARAMETERS FIRST
	//readParameter();
	//std::cout << "focal length " << focal << std::endl;
	//std::cout << "baseline " << baseline << std::endl;

	cv::Mat mat_L, mat_R;
	double dist = 0;
	std::vector<double> distance;
	//cv::undistort(imgLeft_col, mat_L, M1, D1);
	//cv::undistort(imgRight_col, mat_R, M2, D2);
	calDepth();

	if (!vir_cam)
		original_pos = depth_data;
	else
		virtual_pos = depth_data;

	if (!original_pos.empty() && !virtual_pos.empty())
	{
		for (int i = 0; i < 68; i++)
		{
			std::cout << i << "\t ori " << original_pos[i] << "\t vir " << virtual_pos[i] << std::endl;
			dist = sqrt(pow(virtual_pos[i], 2) - pow(original_pos[i], 2));
			std::cout << i << " distance " << dist << std::endl;
			distance.push_back(dist);
		}
		dist = std::accumulate(distance.begin(), distance.end(), 0.0) / distance.size();
		std::cout << "DISTANCE " << dist << std::endl;
	}

}


void FaceDepth::viewSynthesis(void)
{
	
}
