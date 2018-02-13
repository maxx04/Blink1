#include "Servos.h"
#pragma warning(disable : 4996)


Servos::Servos()
{
	sp = new SerialPort(portName);
	Sleep(500);
	sp -> writeSerialPort("#1P1000T300\r\n");
	while (sp -> readSerialPort(m, 2) < 2);
	position = 1000.0f;
	//sp -> writeSerialPort("#1P800T6000\r\n");
	//while (sp -> readSerialPort(m, 2) < 2);
	//sp.writeSerialPort("#1P1800T60\r\n");
	//while (sp.readSerialPort(m, 2) < 2);
}


Servos::~Servos()
{
	position = 1000.0f;
	sprintf(m, "#1P%4.0fT600\r\n", position);
	sp->writeSerialPort(m);
	while (sp->readSerialPort(m, 2) < 2);
}

void Servos::correction(float angle)
{
	if (angle < 100.0f) return;
	position += angle;
	position = (position > 2000) ? 2000 : position;
	position = (position < 500) ? 500 : position;
	sprintf(m, "#1P%4.0fT600\r\n", position);
	sp->writeSerialPort(m);
	while (sp->readSerialPort(m, 2) < 2);
}
