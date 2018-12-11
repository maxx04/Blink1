#include "Motor.h"

int Motor::delay_time = 99; 
bool Motor::speed_changed = false;

// LED-PIN - wiringPi-PIN 0 ist BCM_GPIO 17.
// Wir m�ssen bei der Initialisierung mit wiringPiSetupSys die BCM-Nummerierung verwenden.
// Wenn Sie eine andere PIN-Nummer w�hlen, verwenden Sie die BCM-Nummerierung, und
// aktualisieren Sie die Eigenschaftenseiten � Buildereignisse � Remote-Postbuildereignisbefehl 
// der den GPIO-Export f�r die Einrichtung f�r wiringPiSetupSys verwendet.
#define	LED	4


Motor::Motor(int p1, int p2)
{
	pin1 = p1;
	pin2 = p2;

	wiringPiSetupSys();

	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);

	delay_time = 99;

	th1 = new thread (main_loop, pin1, pin2);

	cout << "Motor started, Id: " << th1->get_id() << endl;
}


Motor::~Motor()
{
}

void Motor::main_loop(int pin1, int pin2)
{
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	
	while (true)
	{
		if (delay_time != 99)
		{
			digitalWrite(pin1, HIGH);	
			delay(delay_time);
		}

		digitalWrite(pin1, LOW);	
		delay(50);
	}	
}

void Motor::rotate(int speed)
{
	delay_time = (int)speed;
}
