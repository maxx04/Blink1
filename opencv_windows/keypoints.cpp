#include "keypoints.h"

keypoints::keypoints()
{
	frame_center = Point2f(0, 0);

	hist_angle = histogram(120, "versatzwinkel");
	hist_length = histogram(50, "versatzlaenge");
	hist_roll = histogram(50, "roll");

	for (size_t i = 0; i < point.size(); i++)	 //OPTI
	{
		point[i].set_flow(Point2f(0, 0));
	}
}

keypoints::~keypoints()
{
	//delete points_queue;
}

void keypoints::clear(void)
{
	point.clear();
	background_points.clear();
}

float keypoints::distance(Point2f a, Point2f b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

float keypoints::length(Point2f a)
{
	return sqrt(pow((a.x), 2) + pow((a.y), 2));
}

Point2f keypoints::get_next_summ_vector()
{
	Point2f sum;

	sum = main_jitter.front();
	main_jitter.pop();

	return sum;
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
	double d = hist_roll.get_main_middle_value(&background_points);

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

	//int i = 0;

	for (int n : background_points)
	{
		p = point[n].get_flow(0); //OPTI mehrmals woanders durchgefuehrt

		if (length(p) == 0.0)
		{
			continue;
		}

		double d = fmodf((atan2(p.x, p.y) * (180.0 / M_PI)) + 360, 360);

		if (p.x != 0.0)	hist_angle.collect(point_satz{ n, (float)d }); // TODO assert

		//i++;
	}

	hist_angle.sort(0, 360);

	//TODO finde hauptdirection: das ist background direction
	// dabei gut zu markieren die punkten die gehoeren main backraund bewegung
	double d = hist_angle.get_main_middle_value(&background_points);

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

	double v = hist_length.get_main_middle_value(&background_points);

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
		main_jitter.push(Point2f(0, 0));
		return	0;
	}

	main_jitter.push(main);

	// Kompensieren jitter
	for (int i = 0; i < point.size(); i++)
	{
		point[i].correct_flow(-main);  // TODO für alle Punkte hinzufügen funktion 
	}

	return 0;
}

void keypoints::calc_distances()
{
	float VFOV2 = 19.1 * M_PI / 180.0; // Vertikale Kameraansichtwinkel geteilt auf  [radian]
	float H = 118.0; // Kameraabstand vom Boden [mm]
	float alfa = 5.1 * M_PI / 180.0;  // Winkel zwischen Bodenebene und Horizotale Kameraebene [radian]
	float V = 720.0; // Anzahl Pixeln vom Bild in vertikale Richtung
	float U = 1280.0; // Anzahl Pixeln vom Bild in horizontaleale Richtung
	float beta, beta1;	// Winkel vom Mittelachse Kamera zu dem Punkt auf dem Boden [radian]
	float gamma;
	float v1;  //Bild koordinate y für vorherige Position
	float distance;	// Abstand vom Kamera zu Punkt auf horizontale Ebene

	//distance_to_cam.clear();
	numbers_of_downpoints.clear();
	//step_length.clear();
	//same_step_pt.clear();
	hist_roll.clear();

	for (int i = 0; i < point.size(); i++)
	{
		float v = point[i].get_position().y; // Bildkoordinaten vom Schlüsselpunkt y
		float u = point[i].get_position().x; // Bildkoordinaten vom Schlüsselpunkt x

		if (v < V / 2) continue; // wenn obere Bildhälfte dan nicht berechnen

		beta = atan((2.0 * v / V - 1.0) * tan(VFOV2)); // OPTI tan_beta lassen / Formel (2) tan(b) = (2*v/V-1)*tan(VFOV/2)

		distance = H / tan(alfa + beta); // OPTI cos(alfa); tan(alfa) vorberechnen	// TODO assert	alfa + beta = pi/2

		//distance_to_cam.push_back(distance);
		numbers_of_downpoints.push_back(i);

		float v1 = point[i].get_position().y;
		//float u1 = prev_points[i].x;

		beta1 = atan((2 * v1 - V) * tan(VFOV2)); // OPTI tan_beta lassen

		float l = distance - H / tan(alfa + beta1);	 // Weg für den Schlüsselpunkt zwischen Frames	// TODO assert	alfa + beta = pi/2

		//hist_distance.collect({ i, l }); //TODO grenzen verfeinern

		//step_length.push_back(l);

		return;
	}
}

void keypoints::draw(cv::Mat* image)
{
	int i = 0;
	float magnify = 1.0;

	for (keypoint p : point)
	{
		circle(*image, (Point)p.position, 3, Scalar(255, 250, 0));
		//line(*image, (Point)p.position, (Point)(magnify * p.l + p.position), Scalar(220, 220, 0), 2);
		//line(*image, (Point)p.position, (Point)(magnify * p.b + p.position), Scalar(0, 220, 200), 2);

		//stringstream text;

		//text << format("%5.1f", p.d);

		//if (p.position.x < (image->cols - 40))
		//{
		//	putText(*image, text.str(), p.position + Point2f(3, -3),
		//		FONT_HERSHEY_PLAIN, 1.0f, Scalar(200, 0, 0), 2);
		//}

		i++;
	}

}

void keypoints::draw_background_points(cv::Mat* image)
{
	for (int n : background_points)
	{
		circle(*image, (Point)point[n].position, 8, Scalar(255, 0, 0), 2);
	}
}

// berechnen geteilte radiale un transitionale Bewegung
void keypoints::calc_distances_1(Point2f fc)
{
	frame_center = fc;
												 
	assert(frame_center.x != 0.0 && frame_center.y != 0.0);

	Point2f v(0, 0);
	//Point2f position(0, 0);
	float length_l, length_r;
	float L = 10.0; // einen Schritt Vorwaerts

	// TODO bereinigen alte positionen

	for (int i = 0; i < point.size(); i++)
	{

		Point2f r = point[i].position - frame_center;
		length_r = length(r);

		v = point[i].flow[0];

		if (r.x != 0.0)
		{
			length_l = length_r * v.x / r.x;

			if (length_r != 0.0)
			{
				point[i].d = L * length_l / length_r + length_l;

				point[i].l = length_l / length_r * r; //TODO andere varianten r = 0 abarbeiten

				point[i].b = v - point[i].l;
			}

		}
		else
		{
			length_l = 0.0;
		}



		//distance_to_cam.push_back(distance);
		//numbers_of_downpoints.push_back(i);

		//hist_distance.collect({ i, l }); //TODO grenzen verfeinern

		//step_length.push_back(length_l);
	}


	//hist_distance.range_max = 100;
	//hist_distance.range_min = -100;
	//hist_distance.sort();

	//float lr = hist_distance.get_main_middle_value(&same_step_pt);	// Mittelwert für Weg

	return;
}
