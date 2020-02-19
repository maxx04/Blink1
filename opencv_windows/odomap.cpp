
#include "odometry.h"
#include "odomap.h"


odomap::odomap()
{
	scale = 0.5f;

	namedWindow(map_window_name, WINDOW_NORMAL | WINDOW_KEEPRATIO);
	
	map = Mat::zeros(1000, 1000, CV_8UC3);
}

void odomap::draw_map(odometry* odm)
{

	 Point2f p0, p1, zero_point;

	 stringstream window_title;

	 map = Mat::zeros(1000, 1000, CV_8UC3);

	p0 = Point2f(map.cols / 2, map.rows / 2);

	zero_point = p0;


	for (Point2f mv : odm -> ego_moving)
	{
		p1.x = p0.x + scale * mv.x; //OPTI verbessern
		p1.y = p0.y - scale * mv.y; //OPTI verbessern

		line(map, (Point)p0, (Point)(p1), Scalar(0, 0, 250), 2);

		p0 = p1;

		if (p0.x < 0.0 || p0.x > map.cols || p0.y < 0.0 || p0.y > map.rows)
		{
			scale /= 2.0f;
			map = Mat::zeros(1000, 1000, CV_8UC3);
			return;
		}

	}

	Point2f p;

	float h; 
	keypoint kp;

	float sin_yaw = sin(odm->yaw_angle * M_PI / 180.0);
	float cos_yaw = cos(odm->yaw_angle * M_PI / 180.0);

	for (int n : odm->kp.ground_points)
	{
		kp = odm-> kp.point[n];

		p.x = odm->current_position.x + kp.rel_ground_pos.x * cos_yaw - kp.rel_ground_pos.y * sin_yaw;
		p.y = - odm->current_position.y - kp.rel_ground_pos.x * sin_yaw - kp.rel_ground_pos.y * cos_yaw;
		h = -kp.rel_ground_pos.z / 10 + 100;

		circle(map, zero_point + scale * p , 2, Scalar(h, h, 0));
	}


	window_title << format("map %.1f / %.1f / %.1f / ", odm->current_position.x, odm->current_position.y, odm->yaw_angle) ;

	setWindowTitle(map_window_name, window_title.str());

	imshow(map_window_name, map);
}


