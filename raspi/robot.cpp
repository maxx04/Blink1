#include <iostream>
#include <string>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define ENCODE_QUALITY 95

#include "../UDP_Base.h"
#include "follower.h"

using namespace cv;
using namespace std;


static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nRobot program start\n"
		"Using OpenCV version " << CV_VERSION << endl;

	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tk - calibrate camera\n"
		<< endl;
}


int main(int argc, char** argv)
{
	help();

	VideoCapture cap;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); //1280
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); //720
	cap.set(cv::CAP_PROP_FPS, 15);

	follower robot;

	cap.open(0);

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	cout << "capturing initialised: " << cap.get(cv::CAP_PROP_FPS) << "  \n";

	static Mat  frame[5], gray;

	vector < int > compression_params;
	int jpegqual = ENCODE_QUALITY; // Compression Parameter
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(jpegqual);


	cout << cap.grab() << " grab result \n";
	cap.retrieve(frame[0]);

	cvtColor(frame[0], gray, COLOR_BGR2GRAY);

	UDP_Base udp_base;

	// Hauptzyclus

	for (;;)
	{
		delay(200); //fuer andere Prozesse und Auslaustung notwendig

		while (udp_base.transfer_busy)
		{
			cout << "waiting transfer \n";
			delay(100);
		}


		if (udp_base.check_incoming_data())
		{
			cout << "new udp data \n";
			robot.new_data_proceed(&udp_base);
			robot.needToInit = true;

			for (int n = 0; n < 5; n++)
			{
				for (int i = 0; i < 5; i++) cap >> frame[n];

				if (robot.proceed_frame(&frame[n])) return 0;
			}

			//delay(1000); // wegen schaerfe

			imencode(".jpg", frame[4], udp_base.encoded, compression_params);
			udp_base.imagegrab_ready = true; // fuer thread mit Server

			cout << endl;
		
		}


	}

	return 0;
}
