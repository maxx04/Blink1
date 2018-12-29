#pragma once
#include <iostream>
#include "Motor.h"
class driver
{
	Motor* motor_links;
	Motor* motor_rechts;

public:
	driver();
	~driver();
	// bewegt sich uengefer distance
	void move(float distance, float duty);
	void change_direction(float angle);
	void ramp(float time, float duty);
	void ramp_down(float time, float duty);
};

