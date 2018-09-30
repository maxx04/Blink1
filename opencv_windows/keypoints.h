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
	
	queue<Point2f>* step_vector_queue; // step_vector von prev_points zu current_points
							// in queue geladen jedes mal nach aufruf load_step_vectors
							
	keypoints();
	~keypoints();
	void clear(void);
	void swap(void);
	int load_step_vectors(void);
	float distance (Point2f a, Point2f b);
	double get_queue_time(void);
	vector <Point2f> * get_next_points_addr(void);

private:




public:
	Point2f get_next_summ_vector();

	// gibt aus ob keine summand vektoren mehr gibts
	bool summ_queue_empty()
	{
		// TODO: Fügen Sie hier Ihren Implementierungscode ein..

		return p_sum.empty();
	}
};

