#include "../SerialPort.h"

SerialPort::SerialPort(const char *portName)
{
	/*
	termios options;
	int myBaud;
	tcgetattr(fd, &options);

	cfmakeraw(&options);
	cfsetispeed(&options, myBaud);
	cfsetospeed(&options, myBaud);

	options.c_cflag |= (CLOCAL | CREAD); // set CLOCAL & CREAD
	options.c_cflag &= ~PARENB; // reset PARENB
	options.c_cflag &= ~CSTOPB; // reset CSTOPB
	options.c_cflag &= ~CSIZE; // reset CSIZE
	options.c_cflag |= CS8; // set an 8-bit word
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // reset these
	options.c_oflag &= ~OPOST; // reset OPOST

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 100;        // Ten seconds (100 deciseconds)

	tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);

	*/
	//ioctl(fd, TIOCMGET, &status);

	//status |= TIOCM_DTR; // set DTR
	//status |= TIOCM_RTS; // set RTS

	//ioctl(fd, TIOCMSET, &status);

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
	//int data = serialDataAvail(fd);

	while (serialDataAvail(fd) > 0 || num >= MAX_DATA_LENGTH)
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
	//serialFlush(fd);

	return true;
}

bool SerialPort::isConnected()
{
	return this->connected;
}
