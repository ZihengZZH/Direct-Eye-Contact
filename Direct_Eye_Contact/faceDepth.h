#pragma once



/* CLASS OF FACE DEPTH MAP */
// this class is responsible for the depth map of human face region
// and it is also responsible for view synthesis

class FaceDepth
{

public:
    // calibration parameters
    cv::Mat M1, M2, D1, D2, R, T, R1, R2, P1, P2, Q;
    double focal, baseline, dispar, depth;
    double cx, cy;
    double max_depth, min_depth;
    bool if_depth = false;

    cv::Mat imgLeft_col, imgRight_col; // undistort frame from web-cam
    dlib::full_object_detection shapes_L, shapes_R;
    std::vector<cv::Point2i> borders = std::vector<cv::Point2i>(4);
    cv::Rect face_rect;

    std::vector<double> depth_data; // depth data of landmarks
    std::vector<double> original_pos, virtual_pos;
    std::vector<std::pair<int, double>> depth_data_index = std::vector<std::pair<int, double>>(68);

    std::vector<cv::Point3f> points_depth = std::vector<cv::Point3f>(76); // for delaunay use
    cv::Mat depth_data_mat;

    enum
    {
        USE_AVERAGE = 0,
        USE_MEDIAN
    };
    int level_method;

    /* View Synthesis */
    cv::Mat Tv = (cv::Mat_<double>(3, 1) << 0.0, -0.25, 0.2); // translation vector
    cv::Vec3f theta = cv::Vec3f(-15, 0, 0); // rotation euler angles
    cv::Mat Rv, RvTv, P;

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
    cv::Rect dlib2opencv(dlib::rectangle r); // key transformation
    bool readParameter(void);
    void disparityMap(int ndisparities, int SADWindowSize);

    bool facialLandmark(bool left); // store the shape of landmarks
    cv::Mat facialLandmarkVis(bool left); // return mat in real-time
    cv::Mat drawLines(void); // visualise the corresponding results

    void levelDepth(void);
    void levelDepthVis(cv::Mat& img, bool if_info);

    bool delaunay(cv::Subdiv2D& subdiv);
    void delaunayDepth(void);
    void delaunayDepthVis(cv::Mat& img);

    void saveFile(cv::Mat img_mat, cv::Rect rect);
    bool calcDepth(void);
    void calcTranslation(bool vir_cam);

    void viewSynthesis(cv::Mat& synthesis_view);

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
    So no need to worry the sensor size

*/