#include "SerialPort.h"

class Servos
{
public:
	float position;
	const char* portName = "\\\\.\\COM7";
	char m[40];
	SerialPort* sp;
	bool in_move;

	Servos();
	~Servos();
	void correction(float angle);
};

