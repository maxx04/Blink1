#include "keypoints.h"



keypoints::keypoints()
{
	step_vector = new vector<Point2f>[MAX_COUNT];
	hist = histogram(120, "winkel");
	hist_l = histogram(50, "length");
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


///

int keypoints::save_step_vectors(void)
{
	Point2f step;
	int number = 0;


	for (size_t i = 0; i < prev_points.size(); i++) 
	{
		//if (status[i] == 1)
		//{
			//speichere in zeit nur gute punkte
			step = (current_points[i] - prev_points[i]);
			number++;
			step_vector[i].push_back(step); // step vektor beladen
		//}

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

double keypoints::get_queue_time(void)
{
	const double timeSec = (frame_timestamp.back() - frame_timestamp.front()) / getTickFrequency();
	// clean
	while (!frame_timestamp.empty()) frame_timestamp.pop();

	return timeSec;
}

vector<Point2f>* keypoints::get_next_points_addr(void)
{
	return &current_points;
}


Point2f keypoints::get_next_summ_vector()
{
	Point2f sum;

	sum = summ_vector.front();
	summ_vector.pop();

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

int keypoints::calculate_move_vectors() // wird jedes frame bearbeitet
{
	Point2f p,main,tmp;

#ifndef _ARM
	const double M_PI = 3.14159265359;
#endif // !ARM


	for (int i = 0; i < prev_points.size(); i++)
	{
		p =  current_points[i] - prev_points[i]; //OPTI mehrmals woanders durchgefuehrt

		double d = fmodf((atan2(p.x, p.y) * (180.0 / M_PI)) + 360, 360);
		if (p.x != 0.0) 
			hist.collect(point_satz{ i , (float)d}); // TODO assert

	}

	hist.range_min = 0;
	hist.range_max = 360;

	hist.sort(); 
	

	//TODO finde hauptdirection: das ist background direction
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung
	double d = hist.get_main_middle_value(&background_points);
	hist.clear();

	//TODO nach histogram vectorlaenge finden hauptvector laenge
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung

	float l;

	for (int i = 0; i < prev_points.size(); i++)
	{
		p = current_points[i] - prev_points[i]; //OPTI mehrmals woanders durchgefuehrt
		l = length(p);
		hist_l.collect(point_satz{ i , (float)l }); //
	}

	hist_l.range_min = hist_l.mean - hist_l.mean * 0.9;
	hist_l.range_max = hist_l.mean + hist_l.mean * 0.9;

	hist_l.sort();

	double v = hist_l.get_main_middle_value(&background_points);

	hist_l.clear();

	main.y = cos(d * M_PI * 2.0 / 360.0 )*v;
	main.x = sin(d * M_PI * 2.0 / 360.0 )*v; //HACK 90 Grad verdreht

	//TODO weitere hauptvectors finden drehen um achse roll

	//TODO bei fortbewegegung ueber raeder drehgeschwindigkeit und zeit zwischen frames
	// kann man finden axiale abstand zwischen frames (gibt es alternativen für quadrakopter?
	// abstand zwischen zwei frames zu finden?)
	// dadurch kann man finden abstand zu keypoints

	//TODO rausnehmen den hauptvector aus punktenbewegung dabei wird man sehen eigene bewegung von punkten


	summ_vector.push(main);


	for (int i = 0; i < prev_points.size(); i++)
	{

		tmp = step_vector[i].back();
		step_vector[i].pop_back();
		step_vector[i].push_back(-(tmp-main)); //TODO Richtung richtig verwenden 

	}
  	return 0;
}
