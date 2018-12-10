#pragma once
#include <wiringPi.h>


class Motor
{
public:
	Motor();
	~Motor();
	void rotate(float speed);
};

