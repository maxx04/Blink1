#include "Servos.h"
#pragma warning(disable : 4996)

//TODO Fehlerbearbeitung realisieren

Servos::Servos()
{
	sp = new SerialPort(portName);
	//	Sleep(500);
	sp -> writeSerialPort("#1P1250T300\r\n");
	sp -> writeSerialPort("#2P1250T300\r\n");
	//  Sleep(500);
	if (sp -> readSerialPort(m, 2) < 1)
		printf("Kein Antwort Servo\r\n");
	position = 1000.0f;
	in_move = false;

}


Servos::~Servos()
{
	position = 1000.0f;
	sp->writeSerialPort("#1P1250T2000\r\n");
	sp->writeSerialPort("#2P1250T2000\r\n");
	sp->writeSerialPort(m);
	if (sp->readSerialPort(m, 2) < 2) printf("Kein Antwort Servo\r\n");
}

void Servos::correction(float angle)
{
	if (sp->readSerialPort(m, 2) < 2 && in_move) return;
	position += angle;
	position = (position > 2000) ? 2000 : position;
	position = (position < 500) ? 500 : position;
	sprintf(m, "#1P%04.0fT600\r\n", position);
	sp->writeSerialPort(m);
	in_move = true;
}

void Servos::move_to_position(float angle)
{
	if (sp->readSerialPort(m, 2) < 2 && in_move) return;
	position = angle;
	position = (position > 2000) ? 2000 : position;
	position = (position < 600) ? 600 : position;
	sprintf(m, "#1P%04.0fT100\r\n", position);
	sp->writeSerialPort(m);
	in_move = true;
}

void Servos::seek()
{
	if (position > 1800.0f)
	{
		position = 1800.0f;
		servo_delta = -servo_delta;
	}

	if (position < 800.0f)
	{
		position = 800.0f;
		servo_delta = -servo_delta;
	}

	position += servo_delta;

	move_to_position(position);
}
