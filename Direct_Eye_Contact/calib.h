#pragma once



/* CLASS OF CAMERA CALIBRATION */
// this class is responsible for the process of stereo camera calibration
// it is one-time in the project, so only parameters are necessary to I/O

class Calibrate
{

public:
    cv::Mat camera_matL;
    cv::Mat camera_matR;
    // 20 pairs of stereo images are recorded
    const int times = 20;
    int time = 0;

private:
    std::vector<std::string> imageList;
    std::string img_list = "calib_xml/imageList.xml";
    std::string cam_intri = "calib_xml/intrinsics.yml";
    std::string cam_extri = "calib_xml/extrinsics.yml";
    std::string face_outL = "test_image/face_rec0_L.jpg";
    std::string face_outR = "test_image/face_rec1_R.jpg";

private:
    float squareSize = 0.025f; // The square size is 2.5cm
    bool displayCorners = false;
    bool useCalibrated = false;
    bool showRectified = false;
    cv::Size boardSize = cv::Size(6, 9); // The chessboard is 7*10
    const int maxScale = 2;

    /*
    squareSize is important for the calibration as it determines the relationship
    the square size is the side length of each square on the chessboard
    in this project, it is 25mm, which is 0.025f
    And the baseline is explicitly measured 10cm / 100mm / 0.1m

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
    /*
        camera matrix M1 M2
        [ fx  0  cx ]
        [ 0  fy  cy ]
        [ 0   0   1 ]
        fx, fy: focal length
        cx, cy: principal point coordinates
    */
    // R1 - output 3*3 rectification transform (rotation matrix) for the first camera
    // R2 - output 3*3 rectification transform (rotation matrix) for the second camera
    // P1 - output 3*4 projection matrix in the new (rectified) coordinate systems for the first camera
    // P2 - output 3*4 projection matrix in the new (rectified) coordinate systems for the second camera
    // Q - output 4*4 disparity-to-depth mapping matrix
    // R - output rotation matrix between the 1st and the 2nd camera coordinate systems
    // T - output translation vector between the coordinate systems of the cameras
    // E - output essential matrix
    // F - output fundamental matrix

public:
    Calibrate() {};
    ~Calibrate() {};

    void displayHelp(void);
    bool readStringList(void);
    void saveImage(void);
    std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize);
    void stereoCalib(void);
    void readParameter(void);
    void rectifyImage(std::string faceL, std::string faceR);


};

