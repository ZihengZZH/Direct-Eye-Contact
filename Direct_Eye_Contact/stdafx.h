// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

// NOTICE
// MFC headers should be before others, in case <windows.h> is included

#pragma once

#include "targetver.h"

// MFC Application
#include <afxwin.h>
#include <afxdialogex.h>
#include <atlimage.h>
#include <afxcontrolbars.h>

// Standard Template Library (STL)
#include <stdio.h>
#include <tchar.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <ctype.h>
#include <cmath>
#include <numeric>

// Parallel computing & OpenMP
#include <ppl.h>
#include <thread>
#include <omp.h>

// OpenCV 3.2
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>

// Dlib 19.9
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <afxcontrolbars.h>



// TODO: reference additional headers your program requires here
