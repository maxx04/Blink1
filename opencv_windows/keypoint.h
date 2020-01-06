#pragma once

#include "opencv2/core.hpp"

const int ANZAHL_VORPOSITIONS = 5;

// Verwaltet einziges Schlüsselpunkt
class keypoint
{
	cv::Point2f position; // Endposition im Bild als float von oberen linken Ecke des Bildes
	cv::Point2f flow[ANZAHL_VORPOSITIONS];	 // Vorherige VERSCHIEBUNGEN des Punktes in vorherigen Aufnahmen 

public:

	keypoint();
	keypoint(cv::Point2f p) { position = p; }
	inline void set_position(cv::Point2f p) { position = p; }
	inline cv::Point2f get_position() { return position; }
	inline void shift_flow() { for (int i = ANZAHL_VORPOSITIONS-1; i > 0; i--) flow[i] = flow[i - 1]; }
	inline void set_flow(cv::Point2f d) { flow[0] = d; }
	inline cv::Point2f get_flow(int i) { CV_Assert(i >= 0 && i < ANZAHL_VORPOSITIONS); return flow[i]; }
	inline void correct_flow(cv::Point2f d) { flow[0] += d; }
};

