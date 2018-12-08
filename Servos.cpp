#include "Servos.h"
#pragma warning(disable : 4996)

//TODO Fehlerbearbeitung realisieren
//TODO Klass als einzelne Servo realisieren

Servos::Servos()
{
	portName = "\\\\.\\COM4";
	servo_delta = 1.0f;
	sp = new SerialPort(portName);
	
	max_position.x = 1900.0;
	min_position.x = 1000.0;
	max_position.y = 1800.0;
	min_position.y = 1200.0;
}


Servos::~Servos()
{
	position = Point2f(Point2f((max_position.x + min_position.x)/2, (max_position.y + min_position.y)/2));
	move_to_position(position);
	wait_on_position(10000);
	sp->~SerialPort();
}

void Servos::test()
{
	in_move = false;
	move_to_position(Point2f((max_position.x + min_position.x) / 2, max_position.y));
	//delay(1000);
	wait_on_position(22000);
	move_to_position(Point2f((max_position.x + min_position.x) / 2, min_position.y));
	//delay(1000);
	wait_on_position(22000);
	position = Point2f((max_position + min_position) / 2);
	move_to_position(position);
	//delay(1000);
	wait_on_position(22000);
	sp->readSerialPort(m);
	cout << m << endl;
	in_move = false;
}

void Servos::correction(Point2f p)
{
//	if (sp->readSerialPort(m, 2) < 2 && in_move) return; // noch in Bewegung
	position += p;
	position.x = (position.x > max_position.x) ? max_position.x : position.x;
	position.x = (position.x < min_position.x) ? min_position.x : position.x;

	position.y = (position.y > max_position.y) ? max_position.y : position.y;
	position.y = (position.y < min_position.y) ? min_position.y : position.y;

	sprintf(m, "#1P%04.0f#2P%04.0fT3000\r\n", position.x, position.y);
	sp->writeSerialPort(m);
	in_move = true;
}

//TODO mit OK kann man grenzen ueberpruefen 

void Servos::move_to_position(Point2f p)
{

	position = p;
	position.x = (position.x > max_position.x) ? max_position.x : position.x;
	position.x = (position.x < min_position.x) ? min_position.x : position.x;

	position.y = (position.y > max_position.y) ? max_position.y : position.y;
	position.y = (position.y < min_position.y) ? min_position.y : position.y;
	sprintf(m, "#1P%04.0f#2P%04.0fT3000\r\n", position.x, position.y);
	sp->writeSerialPort(m);
	in_move = true;
}

bool Servos::wait_on_position(const int time)
{
	const int64 start = getTickCount();
	while (sp->readSerialPort(m) < 2) // Antwort "OK"
	{
		if (getTickCount() - start > (int64)time)
		{
			printf("Kein Antwort Servo - position %f / %f \r\n", position.x, position.y);
			return false;
		}
	}

	return true;
}

void Servos::seek()
{
	if (position.x > 1800.0f)
	{
		position.x = 1800.0f;
		servo_delta = -servo_delta;
	}

	if (position.x < 800.0f)
	{
		position.x = 800.0f;
		servo_delta = -servo_delta;
	}

	position.x += servo_delta;

	move_to_position(position);
}
