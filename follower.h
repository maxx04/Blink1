#pragma once
#pragma warning(disable : 4996)

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "Servos.h"

#include <iostream>
#include <string.h>

using namespace cv;
using namespace std;

class follower
{

	TermCriteria termcrit;
	Size subPixWinSize, winSize;
	const int MAX_COUNT = 500;
	Point2f fokus;
	int number_aim_point = -1;

	Servos s;

	bool needToInit = false;
	bool nightMode = false;

	Mat gray, prevGray, image;

	Mat Affine;

	vector<Point2f> points[2];

	vector<uchar> status;
	vector<float> err;

	vector<Point2f> calc[2];

public:
	follower();
	~follower();
	void init_points();
	void take_picture(Mat* frame);
	void calcOptFlow();
	void transform_Affine();
	void draw();
	void show();
	void swap();
	bool key();
	void look_to_aim();
	int find_nearest_point(Point2f pt);

};

