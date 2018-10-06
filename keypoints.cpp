#include "keypoints.h"



keypoints::keypoints()
{
	step_vector_queue = new queue<Point2f>[MAX_COUNT];
	hist = histogram(60);
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

int keypoints::load_step_vectors(void)
{
	Point2f step,sum = Point2f(0, 0);
	int number = 0;


	for (size_t i = 0; i < prev_points.size(); i++) 
	{
		if (status[i] == 1)
		{
			//speichere in zeit nur gute punkte
			//points_queue[i].push(current_points[i] - prev_points[i] - sum );
			step = (current_points[i] - prev_points[i]);
			sum += step;
			number++;
			step_vector_queue[i].push(step); // step vektor beladen
		}

	}

	if (number != 0) summ_vector.push(sum/(float)number); // TODO Assert 0, summandvektor beladen

	return number;
}

float keypoints::distance(Point2f a, Point2f b)
{
	return sqrt( pow((a.x - b.x),2) + pow((a.y - b.y),2));
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
	Point2f p1 = step_vector_queue[i].front(); //HACK entnahme aus queue vector
	step_vector_queue[i].pop();
	return p1;
}

Point2f keypoints::get_mainmove_backgraund_vector()
{
	return Point2f();
}

int keypoints::calculate_move_vectors()
{
	Point2f p;

	for (int i = 0; i < prev_points.size(); i++)
	{
		p = current_points[i] - prev_points[i]; 
		if (p.x != 0) hist.collect(std::atan2f(p.y, p.x)); // TODO assert
	}

	hist.calculate(); 

	//TODO finde hauptdirection: das ist backgraund direction
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung

	//TODO nach histogram vectorlaenge finden hauptvector laenge
	// dabei gut zu markieren die punkten die gehoeren mein backraund bewegung

	//TODO weitere hauptvectors finden drehen um achse roll

	//TODO bei fortbewegegung ueber raeder drehgeschwindigkeit und zeit zwischen frames
	// kann man finden axiale abstand zwischen frames (gibt es alternativen für quadrakopter?
	// abstand zwischen zwei frames zu finden?)
	// dadurch kann man finden abstand zu keypoints

	//TODO rausnehmen den hauptvector aus punktenbewegung dabei wird man sehen eigene bewegung von punkten





	return 0;
}
