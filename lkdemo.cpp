#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <string.h>

#include "Servos.h"
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
             << endl;
}





int main( int argc, char** argv )
{
  
	double timeSec;

    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");

	VideoCapture cap;

	if (input.size() == 1 && isdigit(input[0]))
		cap.open(0);
    else
        cap.open(input);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

	cout << "capturing initialised \n";

    Mat  frame;
	follower follower_1;


	//const std::string videoStreamAddress = "rtsp://admin:moxter@192.168.178.10/user=admin_password=xhwCY8sx_channel=1_stream=0.sdp?real_stream";

	////open the video stream and make sure it's opened
	//if (!cap.open(videoStreamAddress)) {
	//	std::cout << "Error opening video stream or file" << std::endl;
	//	return -1;
	//}

	// Hauptzyclus

    for(;;)
    {
        cap >> frame;

        if( frame.empty() )  break;


		follower_1.take_picture(&frame);

		follower_1.init_points();

		follower_1.calcOptFlow();

		follower_1.transform_Affine();

		follower_1.draw();

		follower_1.show();

		follower_1.look_to_aim();

		if (follower_1.key()) break;

    }

    return 0;
}
