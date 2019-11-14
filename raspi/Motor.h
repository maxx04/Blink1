#pragma once
#include "i2c.h"
#include <iostream>
#include <thread>
#include <assert.h>
#include <math.h>

#define MAX_MOTORS 2


class Motor
{
	static int number_of_motors;
	int motor_number;
	int pin1, pin2;

	static int* delay_time;
	static bool* direction;

	std::thread* th1;

	static void main_loop(int pin1, int pin2, int motor_number);

public:
	Motor(int p1, int p2);
	~Motor();

	void rotate(float speed);
	void move(float distance);
	void stop() { rotate(0); }
	void test();
};

