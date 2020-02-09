#include "odomap.h"

odomap::odomap()
{
	namedWindow(map_window_name, WINDOW_NORMAL | WINDOW_KEEPRATIO);
	
	map = Mat::zeros(1000, 1000, CV_8UC3);
}

void odomap::draw_map()
{
	 float way = 0;

	 Point2f p0, p1;

	 stringstream window_title;

	 map = Mat::zeros(1000, 1000, CV_8UC3);

	p0 = Point2f(map.cols / 2, map.rows / 2);

	for (diskret_move mv : trajektory)
	{
		p1 = p0 - scale * mv.step; //OPTI verbessern

		line(map, (Point)p0, (Point)(p1), Scalar(0, 0, 250), 2);

		way += p1.y - p0.y;

		p0 = p1;
																					  
		if (p0.x < 0.0 || p0.x > map.cols || p0.y < 0.0 || p0.y > map.rows)
		{
			scale /= 2.0f;
			map = Mat::zeros(1000, 1000, CV_8UC3);
			return;
		}

	}

	window_title << format("map %.1f / ", way);

	setWindowTitle(map_window_name, window_title.str());

	imshow(map_window_name, map);
}

void odomap::add_step(Point2f step)
{
	diskret_move s;
	s.step = step;

	trajektory.push_back(s);
}
