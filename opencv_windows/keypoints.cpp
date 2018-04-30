#include "keypoints.h"



keypoints::keypoints()
{
	points_queue = new queue<Point2f>[MAX_COUNT];
	
}


keypoints::~keypoints()
{
	//delete points_queue;
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

void keypoints::load_queue(void)
{
	Point2f sum = Point2f(0, 0);
	int number = 0;

	for (size_t i = 0; i < prev_points.size(); i++) // TODO
	{
		
		// draw berechnete features
		if (status[i] == 1)
		{
			sum += (current_points[i] - prev_points[i]);
			number++;
		}

	}
	sum /= (float)number;

	p_sum.push( sum );

	for (size_t i = 0; i < prev_points.size(); i++) // TODO
	{
		// draw berechnete features
		if (status[i] == 1)
		{
			//speichere in zeit nur gute punkte
			points_queue[i].push(current_points[i] - prev_points[i] - sum );
		}

	}
}

vector<Point2f>* keypoints::get_next_points_addr(void)
{
	return &current_points;
}
