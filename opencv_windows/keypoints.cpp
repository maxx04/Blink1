#include "keypoints.h"



keypoints::keypoints()
{
	step_vector = new vector<Point2f>[MAX_COUNT];
	hist_angle = histogram(120, "versatzwinkel");
	hist_length = histogram(50, "versatzlaenge");
	hist_distance = histogram(50, "abstand");

	for (size_t i = 0; i < point.size(); i++)
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
	
	prev_points.clear();
	current_points.clear();
}

void keypoints::swap(void)
{
	std::swap(current_points, prev_points); //HACK ob nummerierung passt
}

int keypoints::save_step_vectors(void)
{
	Point2f step;
	int number = 0;


	for (size_t i = 0; i < prev_points.size(); i++) 
	{
			step = current_points[i] - prev_points[i];
			step_vector[i].push_back(step); // step vektor beladen
			number++;
	}

	return number;
}

float keypoints::distance(Point2f a, Point2f b)
{
	return sqrt( pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

float keypoints::length(Point2f a)
{
	return sqrt(pow((a.x), 2) + pow((a.y), 2));
}

vector<Point2f>* keypoints::get_next_points_addr(void)
{
	return &current_points;
}

Point2f keypoints::get_next_summ_vector()
{
	Point2f sum;

	sum = main_jitter.front();
	main_jitter.pop();

	return sum;
}

Point2f keypoints::get_next_step_vector(int i)
{
	Point2f p1 = step_vector[i].back(); //HACK entnahme aus queue vector
	step_vector[i].pop_back();
	return p1;
}

Point2f keypoints::get_mainmove_backgraund_vector()
{
	return Point2f();
}

int keypoints::kompensate_jitter(vector<int>* points_number) // wird jedes frame bearbeitet
{
	Point2f p,main,tmp;

	int i = 0;

	for (int n : *points_number)
	{
		p = point[n].get_flow(0); //OPTI mehrmals woanders durchgefuehrt

		if (length(p) == 0.0)
		{
			continue; 
		}

		double d = fmodf((atan2(p.x, p.y) * (180.0 / M_PI)) + 360, 360);

		if (p.x != 0.0)	hist_angle.collect(point_satz{ i, (float)d }); // TODO assert

		i++;
	}

	hist_angle.range_min = 0;
	hist_angle.range_max = 360;

	hist_angle.sort(); 	

	//TODO finde hauptdirection: das ist background direction
	// dabei gut zu markieren die punkten die gehoeren main backraund bewegung
	double d = hist_angle.get_main_middle_value(&background_points);

	hist_angle.clear();

	for (int n : *points_number)
	{
		p = point[n].get_flow(0);

		double l = length(p);

		hist_length.collect(point_satz{ i, (float)l });

		i++;
	}

	//TODO nach histogram vectorlaenge finden hauptvector laenge
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung

	hist_length.range_min = hist_length.mean - hist_length.mean * 0.9;
	hist_length.range_max = hist_length.mean + hist_length.mean * 0.9;

	hist_length.sort();

	double v = hist_length.get_main_middle_value(&background_points);

	hist_length.clear();

	main.y = cos(d * M_PI * 2.0 / 360.0 )*v;
	main.x = sin(d * M_PI * 2.0 / 360.0 )*v; //HACK 90 Grad verdreht

	// TODO weitere hauptvectors finden drehen um achse roll

	// TODO bei fortbewegegung ueber raeder drehgeschwindigkeit und zeit zwischen frames
	// kann man finden axiale abstand zwischen frames (gibt es alternativen für quadrakopter?
	// abstand zwischen zwei frames zu finden?)
	// dadurch kann man finden abstand zu keypoints

	//TODO rausnehmen den hauptvector aus punktenbewegung dabei wird man sehen eigene bewegung von punkten

	if (v == 0.0)
	{
		main_jitter.push(Point2f(0, 0));
		return	0;
	}

  	main_jitter.push(main);

	// Kompensieren jitter
	for (int i = 0; i < point.size(); i++)
	{
		point[i].correct_flow(-main);
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

	distance_to_cam.clear();
	numbers_of_downpoints.clear();
	step_length.clear();
	same_step_pt.clear();
	hist_distance.clear();

	for (int i = 0; i < current_points.size(); i++)
	{
		float v = current_points[i].y; // Bildkoordinaten vom Schlüsselpunkt y
		float u = current_points[i].x; // Bildkoordinaten vom Schlüsselpunkt x

		if (v < V/2) continue; // wenn obere Bildhälfte dan nicht berechnen

		beta = atan((2.0*v / V - 1.0) * tan(VFOV2)); // OPTI tan_beta lassen / Formel (2) tan(b) = (2*v/V-1)*tan(VFOV/2)
		
		distance = H / tan(alfa + beta); // OPTI cos(alfa); tan(alfa) vorberechnen	// TODO assert	alfa + beta = pi/2

		distance_to_cam.push_back(distance);
		numbers_of_downpoints.push_back(i);

		float v1 = prev_points[i].y;
		//float u1 = prev_points[i].x;

		beta1 = atan((2*v1 - V) * tan(VFOV2)); // OPTI tan_beta lassen

		float l = distance - H / tan(alfa + beta1);	 // Weg für den Schlüsselpunkt zwischen Frames	// TODO assert	alfa + beta = pi/2

		hist_distance.collect({ i, l }); //TODO grenzen verfeinern

		step_length.push_back(l);
	}

	//hist_distance.range_max = 100;
	//hist_distance.range_min = -100;
	hist_distance.sort();
	
	float lr = hist_distance.get_main_middle_value(&same_step_pt);	// Mittelwert für Weg

	return;
}
