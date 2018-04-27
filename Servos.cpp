#include "Servos.h"
#pragma warning(disable : 4996)

//TODO Fehlerbearbeitung realisieren

Servos::Servos()
{
	portName = "\\\\.\\COM7";
	servo_delta = 2.0f;
	sp = new SerialPort(portName);
	max_position.x = 1900.0;
	min_position.x = 1000.0;
	max_position.y = 1900.0;
	min_position.y = 1300.0;
	//	Sleep(500);
	move_to_position(Point2f((max_position.x + min_position.x) / 2, max_position.y));
	wait_on_position(2000);
	move_to_position(Point2f((max_position.x + min_position.x) / 2, min_position.y));
	wait_on_position(2000);
	position = Point2f((max_position + min_position)/2);
	move_to_position(position);
	wait_on_position(2000);
	in_move = false;

}


Servos::~Servos()
{
	position = Point2f(Point2f((max_position.x + min_position.x) / 2, (max_position.y + min_position.y) / 2));
	move_to_position(position);
	if (wait_on_position(1000)) printf("Kein Antwort Servo\r\n");

	sp->~SerialPort();
}

void Servos::correction(Point2f p)
{
	if (sp->readSerialPort(m, 2) < 2 && in_move) return; // noch in Bewegung
	position += p;
	position.x = (position.x > max_position.x) ? max_position.x : position.x;
	position.x = (position.x < min_position.x) ? min_position.x : position.x;

	position.y = (position.y > max_position.y) ? max_position.y : position.y;
	position.y = (position.y < min_position.y) ? min_position.y : position.y;

	sprintf(m, "#1P%04.0f#2P%04.0fT300\r\n", position.x, position.y);
	sp->writeSerialPort(m);
	in_move = true;
}

void Servos::move_to_position(Point2f p)
{
	if (sp->readSerialPort(m, 2) < 2 && in_move) return;
	position = p;
	position.x = (position.x > max_position.x) ? max_position.x : position.x;
	position.x = (position.x < min_position.x) ? min_position.x : position.x;

	position.y = (position.y > max_position.y) ? max_position.y : position.y;
	position.y = (position.y < min_position.y) ? min_position.y : position.y;
	sprintf(m, "#1P%04.0f#2P%04.0fT600\r\n", position.x, position.y);
	sp->writeSerialPort(m);
	in_move = true;
}

bool Servos::wait_on_position(const int time)
{
	const double start = (double)getTickCount();
	while (sp->readSerialPort(m, 2) < 2) // Antwort "OK"
	{
		if ((double)getTickCount() - start > (double)time)
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
