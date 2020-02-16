#include "keypoint.h"


int keypoint::current_step;

keypoint::keypoint()
{
	//current_step = 0;

	position = cv::Point2f(0.0, 0.0);
	for (size_t i = 0; i < flow_steps; i++)
	{
		flow[i] = cv::Point2f(0.0f, 0.0f);
	}
}

inline float keypoint::length(Point2f a)
{
	return sqrt(pow((a.x), 2) + pow((a.y), 2));
}

bool keypoint::check_for_line()
{
	float diff, length_1, length_2;

	if (current_step > 1)	 // 0 index
	{
		length_1 = length(flow[0] - flow[current_step]);

		if (length_1 < 3.0f)  return true; // kleine Vektoren nicht betrachten

		length_2 = length(flow[0] - flow[1]) + length(flow[1] - flow[current_step]);

		if (length_1 != 0.0f)
			diff = (length_2 - length_1) / length_1;
		else
		{
			diff = 0.0f;
			return true;
		}

		//std::cout << length_1 << " | " << length_2 << " | "
		//	<< diff << " | " << current_step << std::endl;

		if (diff > 0.5f)
		{
			return false;
		}
	}

	return true;
}




