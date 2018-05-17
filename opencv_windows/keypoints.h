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
	queue<Point2f> p_sum;
	queue<double> frame_timestamp;
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte
	vector<Point2f> calc[1]; // berechnete punkte
	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK
	
	queue<Point2f>* points_queue; // keypoints in die zeit 

	keypoints();
	~keypoints();
	void clear(void);
	void swap(void);
	void load_queue(void);
	float distance (Point2f a, Point2f b);
	double get_queue_time(void);
	vector <Point2f> * get_next_points_addr(void);

private:




};

