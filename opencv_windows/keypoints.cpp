#include "keypoints.h"

keypoints::keypoints()
{

	hist_angle = histogram(120, "versatzwinkel");
	hist_length = histogram(50, "versatzlaenge");
	hist_roll = histogram(50, "roll");
	hist_step = histogram(20, "step");

	//for (size_t i = 0; i < point.size(); i++)	 //OPTI
	//{
	//	point[i].set_flow(Point2f(0, 0));
	//}
}

keypoints::~keypoints()
{
	//delete points_queue;
}

void keypoints::clear(void)
{
	point.clear();
	keypoint::current_step = 0; // HACK Fuellung ueberwachen
	background_points.clear();
	ground_points.clear();
}

inline float keypoints::distance(Point2f a, Point2f b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

inline float keypoints::length(Point2f a)
{
	return sqrt(pow((a.x), 2) + pow((a.y), 2));
}

int keypoints::kompensate_roll() // wird jedes frame bearbeitet
{

	Point2f p, main, tmp;
	Point2f a_hat, ba, fokus(640, 480);  // TODO
	float l;

	//int i = 0;

	for (int n : background_points)
	{

		p = point[n].get_flow(0); //OPTI mehrmals woanders durchgefuehrt

		if (length(p) == 0.0)
		{
			hist_roll.collect(point_satz{ n, 0.0 });
		}
		else
		{

			tmp = point[n].get_position() - fokus;

			//90 Grad gedreht
			main.x = tmp.y;
			main.y = tmp.x;

			l = length(main);

			if (l != 0.0f)
			{
				ba = (((p.x * main.x + p.y * main.y) / l) / l) * main;

				float h = ba.y / l;  // Moment

				hist_roll.collect(point_satz{ n, h }); // TODO assert
			}
			else
				hist_roll.collect(point_satz{ n, 0.0 });
		}

		//i++;
	}


	hist_roll.sort();

	//TODO finde hauptdirection: das ist background direction
	// dabei gut zu markieren die punkten die gehoeren main backraund bewegung
	double d = hist_roll.main_mean;

	cout << d << endl;

	for (int i = 0; i < point.size(); i++)
	{
		tmp = point[i].get_position() - fokus;
		l = length(tmp);   // hebel

		//90 Grad gedrehte 
		main.x = d * tmp.y;
		main.y = d * tmp.x;

		point[i].correct_flow(-main);  // TODO für alle Punkte hinzufügen funktion 
	}

	hist_roll.clear();

	return 0;

}

int keypoints::kompensate_jitter() // wird jedes frame bearbeitet
{
	Point2f p, main, tmp;

	for (int n : background_points)
	{
		p = point[n].get_flow(0); //OPTI mehrmals woanders durchgefuehrt

		if (length(p) == 0.0)
		{
			continue;
		}

		double d = fmodf((atan2(p.x, p.y) * (180.0 / M_PI)) + 360, 360);

		if (p.x != 0.0)	hist_angle.collect(point_satz{ n, (float)d }); // TODO assert

	}

	hist_angle.sort(0, 360);

	//TODO finde hauptdirection: das ist background direction
	// dabei gut zu markieren die punkten die gehoeren main backraund bewegung
	double d = hist_angle.main_mean;

	hist_angle.clear();

	for (int n : background_points)
	{
		p = point[n].get_flow(0);

		double l = length(p);

		hist_length.collect(point_satz{ n, (float)l });

		//i++;
	}

	//TODO nach histogram vectorlaenge finden hauptvector laenge
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung

	hist_length.range_min = hist_length.mean - hist_length.mean * 0.9;
	hist_length.range_max = hist_length.mean + hist_length.mean * 0.9;

	hist_length.sort(hist_length.mean - hist_length.mean * 0.9,
		hist_length.mean + hist_length.mean * 0.9);

	double v = hist_length.main_mean;

	hist_length.clear();

	main.y = cos(d * M_PI * 2.0 / 360.0) * v;
	main.x = sin(d * M_PI * 2.0 / 360.0) * v; //HACK 90 Grad verdreht

	// TODO weitere hauptvectors finden drehen um achse roll

	// TODO bei fortbewegegung ueber raeder drehgeschwindigkeit und zeit zwischen frames
	// kann man finden axiale abstand zwischen frames (gibt es alternativen für quadrakopter?
	// abstand zwischen zwei frames zu finden?)
	// dadurch kann man finden abstand zu keypoints

	//TODO rausnehmen den hauptvector aus punktenbewegung dabei wird man sehen eigene bewegung von punkten

	// Wenn bewegung ist 0
	if (v == 0.0)
	{
		main_jitter.push_back(Point2f(0, 0));
		return	0;
	}

	main_jitter.push_back(main);

	// Kompensieren jitter
	for (int i = 0; i < point.size(); i++)
	{
		point[i].correct_flow(-main);  // TODO für alle Punkte hinzufügen funktion 
	}

	return 0;
}

int keypoints::check_trajektory()
{

	// die punkte die kann man folgen werden gezählt 
	// und kopiert in point

	int index = 0;


	for (keypoint p : point)
	{

		if (p.check_for_line())
		{
			point[index] = p;

			index++;
		}

	}

	point.resize(index);

	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	return index;

}

void keypoints::draw(cv::Mat* image)
{
	int i = 0;

	for (keypoint p : point)
	{
		//circle(*image, (Point)p.position, 3, Scalar(255, 250, 0));
		//line(*image, (Point)p.position, (Point)(magnify * p.l + p.position), Scalar(220, 220, 0), 1);
		//line(*image, (Point)p.position, (Point)(magnify * p.b + p.position), Scalar(0, 220, 200), 2);

		stringstream text;

		text << format("%5.1f", p.d);

			//putText(*image, text.str(), p.position + Point2f(3, -3),
			//	FONT_HERSHEY_PLAIN, 1.0f, Scalar(0, 0, 0), 1);

		i++;
	}

}

void keypoints::draw_background_points(cv::Mat* image)
{
	for (int n : background_points)
	{
		circle(*image, (Point)point[n].position, 4, Scalar(255, 0, 0), 1);
	}
}

void keypoints::draw_ground_points(cv::Mat* image)
{
	for (int n : ground_points)
	{
		circle(*image, (Point)point[n].position, 4, Scalar(0, 200, 0), 1);
	}

}