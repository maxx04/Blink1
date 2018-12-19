#pragma once
//#pragma warning(disable : 4996)

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <queue>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "../UDP_Base.h"
#include "camera_calibration.h"
#include "neck.h"
#include "driver.h"


using namespace cv;
using namespace std;

class follower
{
	int wait_time = 100;

	neck fneck;
	driver fdriver;

	Mat image;
	Mat cameraMatrix;


public:
	follower();
	~follower();

	void take_picture(Mat* frame);

	bool key(int wait);

	// Bearbeitet jedes frame
	bool proceed_frame(Mat* frame);
	void new_data_proceed(UDP_Base* udp_base);
};


