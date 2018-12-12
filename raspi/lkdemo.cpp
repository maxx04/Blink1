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
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
		cap.set(cv::CAP_PROP_FPS, 90);
	}
    else
        cap.open(input);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

	cout << "capturing initialised: " << cap.get(cv::CAP_PROP_FPS) << "  \n";


    Mat  frame;
	UDP_Base udp_base;
	follower robot;

	// Hauptzyclus
    for(;;)
    {
        cap >> frame;

		if (frame.empty())
		{
			// wenn videodatei in befehlzeile dann beenden.
			if (input.size() != 0) break;
			cap.open(0); //TODO fall mit video berücksichtigen
			cap >> frame;
		}



		if (udp_base.check_incoming_data())
		{
			cout << "new udp data " << udp_base.check_incoming_data() << endl;
			robot.new_data_proceed(&udp_base);
		}

		if (robot.proceed_frame(&frame)) break;

		//TODO skip frames

    }

    return 0;
}
