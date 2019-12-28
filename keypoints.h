#pragma once

#include <iostream>
#include <queue>
#include <string.h>
#include <math.h>

#include "opencv2/core.hpp"
#include "histogram.h"

#ifndef _ARM
const double M_PI = 3.14159265359;
#endif // !ARM

using namespace std;
using namespace cv;

class keypoints
{
private:
	
public:
	const int MAX_COUNT = 500; // Maksimale Anzahl den Punkten die werden berücksichtigt
	queue<Point2f> summ_vector;	// 2d Vektor für resultierende Bildverschiebung
	queue<double> frame_timestamp; // OPTI löschen
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte
	//vector<Point2f> calculated_points[1]; // TODO entfernen? berechnete punkte von vorherigen durch Affine
	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK

	// step_vector von prev_points zu current_points
	// in vector geladen jedes mal nach aufruf load_step_vectors
	vector<Point2f>* step_vector; 

	std::vector<float> distance_to_cam;	// Abstände zu Kamera (Robot) [mm]
	std::vector<float> step_length;	// Berechnete Verschiebung zu Kamera (Schritt pro Frame)
	std::vector<int> numbers_of_downpoints;	// Die Nummern den unteren (Boden) punkten
	std::vector<int> same_step_pt;	// Die Nummern den Punkten mit dem gleichem Schritt

	histogram hist_angle; // Histogram zum Finden vom "background move vector" Winkel
	histogram hist_length; // Histogram zum	Finden vom "background move vector"	Länge
	histogram hist_distance;  // zur Analyse den Abständen
	//TODO spaeter soll man gruppieren punkte und zuordnen zu bewegungsteilen

	vector<int> background_points; // die Punkte die zum Hintergrund gehoeren
					
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
	void calc_distances();
};

