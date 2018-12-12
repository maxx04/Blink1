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
	
	station PC;

	std::string buff;
	net::endpoint ep;

	udata _data;

	Mat* ptrFrame = new Mat(640, 480, CV_16U);

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
     //   cap >> frame;

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

		std::cout << "antwort position : " << _data.dt_udp.servo_position_x << std::endl;

		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl <<
			buff
			<< std::endl
			<< "DATA END" << std::endl;

		//Bild senden

		int n = 15;
		while (n++ < 15)
		{
			sock.recvfrom((char*)ptrFrame, SOCKET_BLOCK_SIZE, &ep);
		}
	

		if (PC.proceed_frame(&frame)) break;

		//TODO skip frames

    }

	sock.close();

    
}
