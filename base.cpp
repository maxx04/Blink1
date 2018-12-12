#include <iostream>
#include <string>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif


#include "station.h"


static void help()
{
    // print a welcome message, and the OpenCV version
    //cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
    //        "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
             << endl;
}



int main( int argc, char** argv )
{

	int wait_time = 0;

    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");

	VideoCapture cap;

    Mat  frame;
	UDP_Base udp_base;
	station PC;



	const std::string videoStreamAddress = "rtsp://admin:xxxx@192.168.178.10/user=admin_password=xhwCY8sx_channel=1_stream=0.sdp?real_stream";
	//open the video stream and make sure it's opened
	if (!cap.open(videoStreamAddress)) 
	{
		std::cout << "Error opening video stream or file" << std::endl;
		return -1;
	}

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
			PC.new_data_proceed(&udp_base);
		}

		if (PC.proceed_frame(&frame)) break;

		//TODO skip frames

    }

    return 0;
}
