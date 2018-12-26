#include "driver.h"

#define MOTOR_RECHTS_FW 23
#define MOTOR_RECHTS_BW 24
#define MOTOR_LINKS_FW 25
#define MOTOR_LINKS_BW 8

using namespace std;

driver::driver()
{
	motor_links = new Motor(MOTOR_RECHTS_FW, MOTOR_RECHTS_BW);
	motor_rechts = new Motor(MOTOR_LINKS_FW, MOTOR_LINKS_BW);

	cout << " driver startet \n";

	//motor_links->test();
	//motor_rechts->test();

}


driver::~driver()
{
}

void driver::move(float distance, float duty)
{
	motor_links->rotate(duty);
	motor_rechts->rotate(duty);
	delay((uint)distance);
	motor_links->stop();
	motor_rechts->stop();
}

void driver::change_direction(float angle)
{
	uint time = (uint)abs(angle)*10; //TODO ermitteln ungaefer

	if (angle == 0.0) return;

	if (angle < 0)
	{
		motor_links->rotate(900);
		delay(time);
		motor_links->stop();
	}
	else
	{
		motor_rechts->rotate(900);
		delay(time);
		motor_rechts->stop();
	}

}
