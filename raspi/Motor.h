#pragma once
#include <wiringPi.h>
#include <iostream>
#include <thread>

using namespace std;
class Motor
{
	int pin1, pin2;
	static int delay_time;
	static bool speed_changed;
	std::thread* th1;

	static void main_loop(int pin1, int pin2);

public:
	Motor(int p1, int p2);
	~Motor();

	void rotate(int speed);
	void stop() { rotate(99); }
};

