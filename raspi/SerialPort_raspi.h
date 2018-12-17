/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/


#ifndef SERIALPORT_H
#define SERIALPORT_H

#define MAX_DATA_LENGTH 255

#include <iostream>

#include <errno.h>
//#include <string.h>

//#include <stdio.h>
//#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>

#include <wiringPi.h>
#include <wiringSerial.h>

class SerialPort
{
private:

	bool connected;
	int fd;

public:

	SerialPort(const char *portName);
	~SerialPort();

	int readSerialPort(char *buffer, unsigned int buf_size = MAX_DATA_LENGTH);
	bool writeSerialPort(char *buffer, unsigned int buf_size);
	bool writeSerialPort(const char *buffer);
	bool isConnected();
};

#endif // SERIALPORT_H

