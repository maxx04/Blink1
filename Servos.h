

#ifndef _Servos
#define _Servos

#include "opencv2\core.hpp"
#include "SerialPort.h"

using namespace cv;
using namespace std;

class Servos
{
public:
	//int N = 0;
	Point2f position;
	Point2f max_position;
	Point2f min_position;
	float servo_delta;
	const char* portName;
	char m[40];
	SerialPort* sp;
	bool in_move;

	Servos();
	~Servos();
	void correction(Point2f p);
	void move_to_position(Point2f p);
	bool wait_on_position(const int time);
	void seek();
};

#endif // !_Servos


