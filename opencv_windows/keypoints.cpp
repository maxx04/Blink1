#include "keypoints.h"



keypoints::keypoints()
{
}


keypoints::~keypoints()
{
}

void keypoints::clear(void)
{
	prev_points.clear();
	current_points.clear();
}

void keypoints::swap(void)
{
	std::swap(current_points, prev_points);
}

vector<Point2f>* keypoints::get_next_points_addr(void)
{
	return &current_points;
}
