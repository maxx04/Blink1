#pragma once

#include "opencv2/core.hpp"

const int ANZAHL_VORPOSITIONS = 5;

using namespace cv;

// Verwaltet einziges Schlüsselpunkt
class keypoint
{
	cv::Point2f position; // Endposition im Bild als float von oberen linken Ecke des Bildes
	cv::Point2f flow[ANZAHL_VORPOSITIONS];	 // Vorherige VERSCHIEBUNGEN des Punktes in vorherigen Aufnahmen 
	cv::Point2f l; // radiale Vektor vom flow[0]
	cv::Point2f b; // verschiebungs Vektor vom flow[0]
	float d;  // Abstand Kamera zu Punkt

public:

	keypoint();
	inline keypoint(Point2f p) 
	{
		position = p; 
		for (size_t i = 0; i < ANZAHL_VORPOSITIONS; i++) flow[i++] = Point2f(0.0f, 0.0f);
	}
	inline void set_position(Point2f p) { position = p; }
	inline Point2f get_position() { return position; }
	inline float get_distance() { return d; }
	inline void shift_flow() { for (int i = ANZAHL_VORPOSITIONS-1; i > 0; i--) flow[i] = flow[i - 1]; }
	inline void set_flow(Point2f d) { flow[0] = d; }
	inline Point2f get_flow(int i) { CV_Assert(i >= 0 && i < ANZAHL_VORPOSITIONS); return flow[i]; }
	inline void correct_flow(Point2f d) { flow[0] += d; }
	inline Point2f get_full_flow()
	{
		Point2f p(0, 0);
		for (size_t i = 0; i < ANZAHL_VORPOSITIONS; i++) p += flow[i];
		return p;
	}
	inline void set_vectors(Point2f _l, Point2f _b, float _d) { l = _l; b = _b; d = _d; }

	friend class keypoints;

};

