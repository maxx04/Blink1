#include <iostream>
#include <string>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define ENCODE_QUALITY 50

#include "../UDP_Base.h"
#include "follower.h"

using namespace cv;
using namespace std;


static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		"\ts - set aim point\n"
		"\tk - calibrate camera\n"
		<< endl;
}





int main(int argc, char** argv)
{

	int wait_time = 0;

	//   help();
	   //cv::CommandLineParser parser(argc, argv, "{@input|0|}");
	   //string input = parser.get<string>("@input");

	VideoCapture cap;

	//if (input.size() == 1 && isdigit(input[0]))
	//{
	//	cap.open(0);
	//	//cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	//	//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	//	//cap.set(cv::CAP_PROP_FPS, 90); // OPTI
	//}
 //   else
 //       cap.open(input);

	cap.open(0);
	cap.set(cv::CAP_PROP_FPS, 15);

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	cout << "capturing initialised: " << cap.get(cv::CAP_PROP_FPS) << "  \n";

	static Mat  frame, gray;
	
	vector < int > compression_params;
	int jpegqual = ENCODE_QUALITY; // Compression Parameter
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(jpegqual);
	

	cout << cap.grab() << " grab result \n";
	cap.retrieve(frame);

	//imwrite("test.jpg", frame);
	cvtColor(frame, gray, COLOR_BGR2GRAY);


	//for (int i = 0; i < 6; i++) cap >> frame;

	//imshow("LK Demo", gray);
	//waitKey(100);

	UDP_Base udp_base(&gray);
	follower robot;

	// Hauptzyclus

	for (;;)
	{

		if (udp_base.check_incoming_data())
		{
			cout << "new udp data \n";
			robot.new_data_proceed(&udp_base);
			cout << "aufnahme \n";

			for (int i = 0; i < 6; i++) cap >> frame;

			if (cap.read(frame))
			{
				cout << "aufgenommen \n";
			}
			else
			{
				cout << "nicht aufgenommen \n";
			}

			cvtColor(frame, gray, COLOR_BGR2GRAY);
			imencode(".jpg", frame, udp_base.encoded, compression_params);

			udp_base.imagegrab_ready = true; // fuer thread mit Server



			//imwrite("test.jpg", frame);
			cout << "grab true \n";
			imshow("LK Demo", frame);
			waitKey(200);



			while (udp_base.transfer_busy)
			{
				cout << "waiting transfer \r";
			}

			if (robot.proceed_frame(&gray)) return 0;
		}



		//TODO skip frames

	}

	return 0;
}
