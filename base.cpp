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

    Mat  frame, im8u;


	//cap.open(0);

	//if (!cap.isOpened())
	//{
	//	cout << "Could not initialize capturing...\n";
	//	return 0;
	//}
	


	Mat* ptr_in_Frame = new Mat(480, 640, CV_8U);

	std::string buff;
	net::endpoint ep;

	udata _data;
	//	station PC;

	//we must call net::init() on windows, if not on windows it is a no-op
	net::init();

	//create a socket without binding in the ctor
	net::socket sock(net::af::inet, net::sock::dgram, 4010);

	ep = net::endpoint("192.168.178.41", 8080);

	//most calls in xsocket return the same value as there c counterparts
	//like so if sock.bind returns -1 it failed
	int r = sock.connect(ep);
	if (r == -1) {
		perror("failed to connect");
		return -1;
	}

	std::cout << "socket bound to: " << sock.getlocaladdr().to_string() << std::endl;

	sock.send("connect", 512);

	Sleep(1000);


	//const std::string videoStreamAddress = "rtsp://admin:xxxx@192.168.178.10/user=admin_password=xhwCY8sx_channel=1_stream=0.sdp?real_stream";
	////open the video stream and make sure it's opened
	//if (!cap.open(videoStreamAddress)) 
	//{
	//	std::cout << "Error opening video stream or file" << std::endl;
	//	return -1;
	//}

	// Hauptzyclus

    for(;;)
    {
       cap >> frame;

		//if (frame.empty())
		//{
		//	// wenn videodatei in befehlzeile dann beenden.
		//	if (input.size() != 0) break;
		//	cap.open(0); //TODO fall mit video berücksichtigen
		//	cap >> frame;
		//}

		std::cin >> wait_time;

		sock.send(_data.union_buff, 512);

		int i = sock.recvfrom(_data.union_buff, 512, &ep);

		if (i == -1)	break;

		std::cout << "packet from: " << ep.to_string() << std::endl;
			//<< "DATA START" << std::endl <<
			//buff
			//<< std::endl
			//<< "DATA END" << std::endl;

		//Bild senden

		int n = 0;

		size_t n_blocks = ptr_in_Frame->total() * ptr_in_Frame->elemSize() / SOCKET_BLOCK_SIZE;

		if (ptr_in_Frame->isContinuous())
		{
			cout << "is Continuous" << endl;
		}

		char* data_start = (char*)(ptr_in_Frame->data);

		cout << "start transfer \t" << hex << int(data_start) << dec << endl;

		while (n < n_blocks)
		{
			//cout << n << "\r";
			sock.recvfrom(data_start + n * SOCKET_BLOCK_SIZE, SOCKET_BLOCK_SIZE, &ep);

			sock.send("ready", 512);
			n++;
		}

		cout << "end transfer " << n << " blocks" << endl;
	
		imshow("LK", *ptr_in_Frame);
		waitKey(100);

//		if (PC.proceed_frame(ptr_in_Frame)) break;

		//TODO skip frames

    }

	sock.close();

    
}
