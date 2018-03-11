#include "SerialPort.h"

#ifndef _Servos
#define _Servos
class Servos
{
public:
	float position;
	float servo_delta = 6.0f;
	const char* portName = "\\\\.\\COM7";
	char m[40];
	SerialPort* sp;
	bool in_move;

	Servos();
	~Servos();
	void correction(float angle);
	void move_to_position(float angle);
	void seek();
};

#endif // !_Servos


