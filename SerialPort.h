/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/


#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 3000
#define MAX_DATA_LENGTH 255

#ifndef _ARM
#include <windows.h>
#else
#include <errno.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <termios.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#endif // !_ARM

	#include <stdio.h>
	#include <stdlib.h>

class SerialPort
{
private:

	bool connected;


#ifndef _ARM
	HANDLE handler;
	COMSTAT status;
	DWORD errors;
	DWORD bytesSend;
#else
	int fd;
#endif !_ARM
public:

	SerialPort(const char *portName);
	~SerialPort();

	int readSerialPort(char *buffer, unsigned int buf_size);
	bool writeSerialPort(char *buffer, unsigned int buf_size);
	bool writeSerialPort(const char *buffer);
	bool isConnected();
};

#endif // SERIALPORT_H

