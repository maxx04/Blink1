#include "../SerialPort.h"

SerialPort::SerialPort(const char *portName)
{
	this->connected = false;

	if ((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return;
	}

	fprintf(stderr, "serial device opened \n");

	this->connected = true;
}

SerialPort::~SerialPort()
{
	if (this->connected) 
	{
		this->connected = false;
		serialClose(fd);
	}
}

int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{

	int num = 0;
	int data = serialDataAvail(fd);

	while (serialDataAvail(fd) > 0 || num >= 15 )
	{
		buffer[num++] = serialGetchar(fd);
//		if (buffer[num - 1] == 'O' && buffer[num] == 'K') return num;
	}
	return num;
}

bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
{
	//TODO fehlerbearbeitung

	serialPuts(fd, buffer);

	return true;
}

bool SerialPort::writeSerialPort(const char *buffer)
{
	serialPuts(fd, buffer);
	serialFlush(fd);

	return true;
}

bool SerialPort::isConnected()
{
	return this->connected;
}
