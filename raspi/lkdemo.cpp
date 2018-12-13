#include <iostream>
#include <string>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

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





int main( int argc, char** argv )
{

	int wait_time = 0;

    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");

	VideoCapture cap;

	if (input.size() == 1 && isdigit(input[0]))
	{
		cap.open(0);
		//cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
		//cap.set(cv::CAP_PROP_FPS, 90); // OPTI
	}
    else
        cap.open(input);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

	cout << "capturing initialised: " << cap.get(cv::CAP_PROP_FPS) << "  \n";

	static Mat  frame, gray;

	cap >> frame;
	cvtColor(frame, gray, COLOR_BGR2GRAY);

	//imshow("LK Demo", frame);
	//waitKey(100);

	int k;

	cin >> k;

	UDP_Base udp_base(&gray);
	follower robot;

	// Hauptzyclus

    for(;;)
    {

		if (udp_base.check_incoming_data())
		{

			//imshow("LK Demo", *udp_base.get_frame_pointer());
			//waitKey(100);
			cout << "new udp data " << udp_base.check_incoming_data() << endl;
			robot.new_data_proceed(&udp_base);

			while (!udp_base.transfer_busy) 
			{
				cout << "waiting transfer \r";
			}
				cap >> frame;

				cvtColor(frame, gray, COLOR_BGR2GRAY);

			if (robot.proceed_frame(&gray)) return 0;
		}



		//TODO skip frames

    }

    return 0;
}
