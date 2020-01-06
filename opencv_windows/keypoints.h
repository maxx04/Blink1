#pragma once

#include <iostream>
#include <queue>
#include <string.h>
#include <math.h>
#include "opencv2/core.hpp"
#include "keypoint.h"
#include "histogram.h"

#ifndef _ARM
const double M_PI = 3.14159265359;
#endif // !ARM

using namespace std;
using namespace cv;


// Klass keypoints verwaltet Schlüsselpunkte mit deren Fluss
// Schlüsselpunkt verwaltet:
// 1 - "geschichte" vorherige punkte innerhalb einen satz (Anzahl "frames" für Ermittlung des Flusses)
// 2 - berechnet Fluss
// 3 - verwaltet Statistic

class keypoints
{
private:
	
public:
	const int MAX_COUNT = 800; // Maksimale Anzahl den Punkten die werden berücksichtigt
	queue<Point2f> main_jitter;	// Reihe für 2d Vektors den resultierendes Bildverschiebung
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte

	vector<keypoint> point; // Schlüsselpunkte selber

	// step_vector von prev_points zu current_points
	// in vector geladen jedes mal nach aufruf load_step_vectors
	vector<Point2f>* step_vector; 

	std::vector<float> distance_to_cam;	// Abstände zu Kamera (Robot) [mm]
	std::vector<float> step_length;	// Berechnete Verschiebung zu Kamera (Schritt pro Frame)
	std::vector<int> numbers_of_downpoints;	// Die Nummern den unteren (Boden) punkten
	std::vector<int> same_step_pt;	// Die Nummern den Punkten mit dem gleichem Schritt
	vector<int> background_points; // die Punkte die zum Hintergrund gehoeren

	histogram hist_angle; // Histogram zum Finden vom "background move vector" Winkel
	histogram hist_length; // Histogram zum	Finden vom "background move vector"	Länge
	histogram hist_distance;  // zur Analyse den Abständen
	//TODO spaeter soll man gruppieren punkte und zuordnen zu bewegungsteilen


					
	keypoints();
	~keypoints();

	void clear(void);
	void swap(void);
	int save_step_vectors(void);
	float distance (Point2f a, Point2f b);
	float length(Point2f a);  // Vektorlänge
	vector <Point2f> * get_next_points_addr(void);

	Point2f get_next_summ_vector();

	// gibt aus ob keine summand vektoren mehr gibts
	bool summ_queue_empty() { return main_jitter.empty(); }

	Point2f get_next_step_vector(int i);

	// gibt aus ob keine summand vektoren mehr gibts
	inline bool step_vector_empty(int i) { return step_vector[i].empty(); }

	Point2f get_mainmove_backgraund_vector();

	int kompensate_jitter(vector<int>* points_number);
	void calc_distances();
};

