#pragma once

#include <iostream>
#include <queue>
#include <string.h>

#include "opencv2/core.hpp"

using namespace cv;
using namespace std;

class keypoints
{
public:
	const int MAX_COUNT = 300;
	//vector<Point2f> points[2];
	vector<Point2f> prev_points;
	vector<Point2f> current_points;
	vector<Point2f> calc[1]; // berechnete punkte
	vector<uchar> status;
	vector<float> err;

	keypoints();
	~keypoints();
	void clear(void);
	void swap(void);
	vector <Point2f> * get_next_points_addr(void);

private:
	

	queue<Point2f> points_queue[10]; // keypoints in die zeit 0 bis 9

};

