#include <wiringPi.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// LED-PIN - wiringPi-PIN 0 ist BCM_GPIO 17.
// Wir müssen bei der Initialisierung mit wiringPiSetupSys die BCM-Nummerierung verwenden.
// Wenn Sie eine andere PIN-Nummer wählen, verwenden Sie die BCM-Nummerierung, und
// aktualisieren Sie die Eigenschaftenseiten – Buildereignisse – Remote-Postbuildereignisbefehl 
// der den GPIO-Export für die Einrichtung für wiringPiSetupSys verwendet.
#define	LED	17

//wiringPiSetupSys();

	//pinMode(LED, OUTPUT);

	//while (true)
	//{
	//	digitalWrite(LED, HIGH);  // Ein
	//	delay(500); // ms
	//	digitalWrite(LED, LOW);	  // Aus
	//	delay(500);
	//}

	using namespace cv;
	using namespace std;

	int main(int argc, char ** argv)
	{
		int m = 1;
		VideoCapture cap(0);
		if (!cap.isOpened()) {
			cerr << "ERROR: Unable to open the camera" << endl;
			return 0;
		}

		Mat frame;
		cout << "Start grabbing, press a key on Live window to terminate" << endl;
		while (1) {
			const double start = (double)getTickCount();
			cap >> frame;
			m++;
			if (frame.empty()) {
				cerr << "ERROR: Unable to grab from the camera" << endl;
				break;
			}
			imshow("Live", frame);
			const double timeSec = (getTickCount() - start) / getTickFrequency();
 			cout << m << " " << timeSec << endl;

			int key = cv::waitKey(100);
			key = (key == 255) ? -1 : key; //#Solve bug in 3.2.0
			if (key >= 0)
				break;
		}

		cout << "Closing the camera" << endl;
		cap.release();
		destroyAllWindows();
		cout << "bye!" << endl;
		return 0;
	}






