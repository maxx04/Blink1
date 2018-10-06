#pragma once

#include <iostream>
#include <queue>
#include <string.h>

#include "opencv2/core.hpp"
#include "histogram.h"

using namespace cv;
using namespace std;

class keypoints
{
private:
	;
public:
	const int MAX_COUNT = 300;
	queue<Point2f> summ_vector;
	queue<double> frame_timestamp;
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte
	vector<Point2f> calculated_points[1]; // berechnete punkte von vorherigen durch Affine
	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK
	queue<Point2f>* step_vector_queue; // step_vector von prev_points zu current_points
							// in queue geladen jedes mal nach aufruf load_step_vectors

	histogram hist; //histogramm zum finden vom backgraund move vector
	//TODO spaeter soll man gruppieren punkte und zuordnen zu bewegungsteilen
							
	keypoints();
	~keypoints();
	void clear(void);
	void swap(void);
	int load_step_vectors(void);
	float distance (Point2f a, Point2f b);
	double get_queue_time(void);
	vector <Point2f> * get_next_points_addr(void);

	Point2f get_next_summ_vector();

	// gibt aus ob keine summand vektoren mehr gibts
	bool summ_queue_empty() { return summ_vector.empty(); }

	Point2f get_next_step_vector(int i);

	// gibt aus ob keine summand vektoren mehr gibts
	bool step_vector_empty(int i){ return step_vector_queue[i].empty(); }

	Point2f get_mainmove_backgraund_vector();

	// berechnet bewegugsvectoren pro schritt TODO Batch 
	int calculate_move_vectors();
};

