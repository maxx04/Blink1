#pragma once
#include <wiringPi.h>


class Motor
{
	int pin1, pin2;
	static int delay_time;
	static bool speed_changed;

public:
	Motor(int p1, int p2);
	~Motor();
	void zyclus();
	void rotate(int speed);
	void stop() { rotate(999); }
};

