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


// Klass keypoints verwaltet Schl�sselpunkte mit deren Fluss
// Schl�sselpunkt verwaltet:
// 1 - "geschichte" vorherige punkte innerhalb einen satz (Anzahl "frames" f�r Ermittlung des Flusses)
// 2 - berechnet Fluss
// 3 - verwaltet Statistic

class keypoints
{
	//friend class keypoint;

private:
	Point2f frame_center;
	
public:
	const int MAX_COUNT = 800; // Maksimale Anzahl den Punkten die werden ber�cksichtigt
	queue<Point2f> main_jitter;	// Reihe f�r 2d Vektors den resultierendes Bildverschiebung
	vector<keypoint> point; // Schl�sselpunkte selber
	vector<int> background_points; // die Punkte die zum Hintergrund gehoeren
	vector<int> ground_points; // die Punkte die zum Boden gehoeren um (angenommen Ebene!)

	histogram hist_angle; // Histogram zum Finden vom "background move vector" Winkel
	histogram hist_length; // Histogram zum	Finden vom "background move vector"	L�nge
	histogram hist_roll;  // zur Analyse den Abst�nden
	histogram hist_step;  // zur Analyse den Schrittl�nge
	//TODO spaeter soll man gruppieren punkte und zuordnen zu bewegungsteilen

	keypoints();
	~keypoints();

	void clear(void);
	float distance (Point2f a, Point2f b);
	float length(Point2f a);  // Vektorl�nge

	Point2f get_next_summ_vector();

	// gibt aus ob keine summand vektoren mehr gibts
	bool summ_queue_empty() { return main_jitter.empty(); }
	void set_fokus(Point2f f); // zuweisen Bildmitte (hoehe / 2, breite / 2)
 	int kompensate_roll();
	int kompensate_jitter();
	void calc_distances(float step);  // berechnet distanz vom kamera zu punkten vor dem step Bewegung
	float calc_step();
	void draw(cv::Mat* image);
	void draw_background_points(cv::Mat* image);
	void draw_ground_points(cv::Mat* image);
};

