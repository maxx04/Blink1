#include <iostream>
#include <string>

#include "../UDP_Base.h"
#include "follower.h"
#include "robot.h"

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

static UDP_Base udp_base;

int main(int argc, char** argv)
{
	help();

	TickMeter tm;
	follower robot;

	//VideoCapture cap;

	//cap.open(0);

	//cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); //1280
	//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); //720
	////cap.set(cv::CAP_PROP_FPS, 15);

	 //std::cout << cv::getBuildInformation() << std::endl; 

	const char* gst = "nvarguscamerasrc  ! video/x-raw(memory:NVMM), format=(string)NV12,  \
		width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! \
			nvvidconv         ! video/x-raw,              format=(string)BGRx ! \
			videoconvert      ! video/x-raw,              format=(string)BGR  ! appsink";

	cv::VideoCapture cap(gst);


	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	cout << "capturing initialised: " << endl;

	cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << ":" <<
		cap.get(cv::CAP_PROP_FRAME_HEIGHT) << ":" <<
		cap.get(cv::CAP_PROP_FPS) << endl;

	static Mat  frame[ANZAHL_AUFNAHMEN], gray;

	vector < int > compression_params;
	int jpegqual = ENCODE_QUALITY; // Compression Parameter
	//compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	//compression_params.push_back(jpegqual);				   //TODO

	cout << "waiting transfer" << endl;

/*

	cout << cap.grab() << " grab result \n";
	cap.retrieve(frame[0]);

	cvtColor(frame[0], gray, COLOR_BGR2GRAY);

*/

	// Hauptzyclus

	for (;;)
	{
		delay(200); //fuer andere Prozesse und Auslaustung notwendig


		while (udp_base.transfer_busy)
		{
			delay(200);
		}



		if (udp_base.check_incoming_data())
		{
			cout << "new udp data \n";
			// start der bewegung
			tm.reset();
			tm.start();

			robot.new_data_proceed(&udp_base);
			robot.needToInit = true;

			

			// serie von bildern aufnehmen
			for (int n = 0; n < ANZAHL_AUFNAHMEN; n++)
			{
				robot.stop_move(&udp_base);
				//  buffer entlehren
				delay(800);
				for (int i = 0; i < 5; i++) cap >> frame[n];

				cap >> frame[n];
				tm.stop();

				cout << "aufnahmen abstand " << tm << " s" << endl;
				tm.reset();
				tm.start();
				delay(10);

				robot.start_move(&udp_base);
			}

			robot.stop_move(&udp_base);

		
			//bearbeiten
			for (int n = 0; n < ANZAHL_AUFNAHMEN; n++)
			{
				// wenn alles in ordnung weiter
				if (robot.proceed_frame(&frame[n], n))
				{
					return 11;
				}
				cout << "Frame " << n << endl; ;
			}

			
			

			imencode(".jpg", frame[ANZAHL_AUFNAHMEN-1], udp_base.encoded, compression_params);

			robot.copy_keypoints();

			udp_base.imagegrab_ready = true; // fuer thread mit Server

		}


	}

	cap.release();
	return 0;
}
