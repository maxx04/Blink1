#include "SerialPort.h"

class Servos
{
public:
	float position;
	const char* portName = "\\\\.\\COM5";
	char m[40];
	SerialPort* sp;

	Servos();
	~Servos();
	void correction(float angle);
};

