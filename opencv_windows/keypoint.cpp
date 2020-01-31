#include "keypoint.h"

keypoint::keypoint()
{
	position = cv::Point2f(0.0, 0.0);
	for (size_t i = 0; i < ANZAHL_VORPOSITIONS; i++)
	{
		flow[i] = cv::Point2f(0.0f, 0.0f);
	}
}



