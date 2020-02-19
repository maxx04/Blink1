#pragma once

#include <iostream>
#include "opencv2/core.hpp"

const int flow_steps = 5;

using namespace cv;

// Verwaltet einziges Schlüsselpunkt
class keypoint
{
	static int current_step;

	cv::Point2f position; // Endposition im Bild als float von oberen linken Ecke des Bildes
	cv::Point2f flow[flow_steps];	 // Vorherige VERSCHIEBUNGEN des Punktes in vorherigen Aufnahmen 
	cv::Point2f l; // radiale Vektor vom flow[0]
	cv::Point2f b; // verschiebungs Vektor vom flow[0]

public:

	cv::Point3f rel_ground_pos;	// 	Koordinaten Keypunkten zu Roboter(Cam)
	float d;  // relatives abstand, wenn auf Schritt multiplizieren wird Abstand Kamera zu Punkt

	keypoint();
	bool check_for_line(); // pruefen ob Flow in bestimmten Bereich ist
	inline float length(Point2f a);  // Vektorlaenge
	inline keypoint(Point2f p) 
	{
		position = p; 
		for (size_t i = 0; i < flow_steps; i++) flow[i++] = Point2f(0.0f, 0.0f);
	}
	inline void set_position(Point2f p) { position = p; }
	inline Point2f get_position() { return position; }
	inline float get_distance() { return d; }
	inline void set_step() 
	{ 
		if (current_step < flow_steps)
		{
			current_step++;
		}
	}
	inline void shift_flow() 
	{
		for (int i = flow_steps - 1; i > 0; i--) flow[i] = flow[i - 1];
	}
	inline void set_flow(Point2f d) { flow[0] = d; }
	inline Point2f get_flow(int i) { CV_Assert(i >= 0 && i < flow_steps); return flow[i]; }
	inline void correct_flow(Point2f d) { flow[0] += d; }
	inline Point2f get_full_flow()	 // Gesamtverschiebung allen Schritten.  	
	{
		Point2f p(0, 0);
		for (size_t i = 0; i < flow_steps; i++) p += flow[i];
		return p;
	};
	// speichern Vektoren
	inline void set_vectors(Point2f translations_vector, Point2f quer_vector, float distance)
	{
		l = translations_vector; b = quer_vector; d = distance;
	}; 

	friend class keypoints;

};

