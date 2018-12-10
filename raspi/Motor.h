#pragma once
#include <wiringPi.h>


class Motor
{
	int pin1, pin2;
public:
	Motor(int p1, int p2);
	~Motor();
	void rotate(float speed);
};

