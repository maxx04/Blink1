#include "Motor.h"


// LED-PIN - wiringPi-PIN 0 ist BCM_GPIO 17.
// Wir müssen bei der Initialisierung mit wiringPiSetupSys die BCM-Nummerierung verwenden.
// Wenn Sie eine andere PIN-Nummer wählen, verwenden Sie die BCM-Nummerierung, und
// aktualisieren Sie die Eigenschaftenseiten – Buildereignisse – Remote-Postbuildereignisbefehl 
// der den GPIO-Export für die Einrichtung für wiringPiSetupSys verwendet.
#define	LED	4


Motor::Motor(int p1, int p2)
{
	pin1 = p1;
	pin2 = p2;

	wiringPiSetupSys();

	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
}


Motor::~Motor()
{
}

void Motor::rotate(float speed)
{
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);

	if (speed = 0)
	{
		digitalWrite( pin1, LOW);
		digitalWrite( pin2, LOW);

		return;
	}
	

		digitalWrite(pin1, HIGH);  // Ein
		delay(500); // ms
		digitalWrite(pin1, LOW);	  // Aus
		delay(500);
	
}
