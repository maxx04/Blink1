#pragma once

#include <iostream>
#include <queue>
#include <string.h>

#include "opencv2/core.hpp"
#include "histogram.h"

using namespace std;
using namespace cv;

class keypoints
{
private:
	
public:
	const int MAX_COUNT = 500;
	queue<Point2f> summ_vector;
	queue<double> frame_timestamp;
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte
	//vector<Point2f> calculated_points[1]; // TODO entfernen? berechnete punkte von vorherigen durch Affine
	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK

	vector<Point2f>* step_vector; // step_vector von prev_points zu current_points
							// in vector geladen jedes mal nach aufruf load_step_vectors

	histogram hist; //histogramm zum finden vom background move vector
	histogram hist_l;
	//TODO spaeter soll man gruppieren punkte und zuordnen zu bewegungsteilen

	vector<int> background_points; // die punkten die zum hintergrund gehoeren
					
	keypoints();
	~keypoints();
	void clear(void);
	void swap(void);
	int save_step_vectors(void);
	float distance (Point2f a, Point2f b);
	float length(Point2f a);
	double get_queue_time(void);
	vector <Point2f> * get_next_points_addr(void);

	Point2f get_next_summ_vector();

	// gibt aus ob keine summand vektoren mehr gibts
	bool summ_queue_empty() { return summ_vector.empty(); }

	Point2f get_next_step_vector(int i);

	// gibt aus ob keine summand vektoren mehr gibts
	bool step_vector_empty(int i){ return step_vector[i].empty(); }

	Point2f get_mainmove_backgraund_vector();

	// berechnet bewegugsvectoren pro schritt TODO Batch 
	int calculate_move_vectors();
};

