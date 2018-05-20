#pragma once
#pragma warning(disable : 4996)

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

#include "Servos.h"
#include "opencv_windows/keypoints.h"
#include "camera_calibration/camera_calibration.h"

using namespace cv;
using namespace std;

class follower
{
	const int queue_size = 10;
	TermCriteria termcrit;
	Size subPixWinSize, winSize;
	
	float pixel_pro_step = 8.0;

	Point2f fokus;
	int number_aim_point = -1;
	double frame_time = 0.0;

	Servos s;

	bool needToInit = false;
	bool nightMode = false;

	Mat gray, prevGray, image;

	Mat Affine;
	keypoints kp; // keypoints in zeit

public:
	follower();
	~follower();
	void init_points();
	void take_picture(Mat* frame);
	void calcOptFlow();
	void transform_Affine();
	int draw();
	void show();
	void cam_calibrate();
	void swap();
	bool key(int wait);
	void look_to_aim();
	int find_nearest_point(Point2f pt);

};

