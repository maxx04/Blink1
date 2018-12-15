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

	Mat* ptr_in_Frame = new Mat(480, 640, CV_8U);
	static std::vector < uchar > encoded(65536);

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

	sock.send("connect", 8);

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

		cout << " x:"; cin >> _data.dt_udp.servo_position_x;
		cout << " y:"; cin >> _data.dt_udp.servo_position_y;

		sock.send(_data.union_buff, SOCKET_BLOCK_SIZE);

		union int_char
		{
			int nb;
			char bf[sizeof(int)];
		} tmp;

		int i = sock.recvfrom(tmp.bf, sizeof(int), &ep); // 1. antwort

		if (i == -1)	break;

		std::cout << "packet from: " << ep.to_string() << std::endl;

		//Bild senden

		int n = 0;

		//size_t n_blocks = ptr_in_Frame->total() * ptr_in_Frame->elemSize() / SOCKET_BLOCK_SIZE;
		size_t n_blocks = tmp.nb; // 1 + (_encoded.size() - 1) / SOCKET_BLOCK_SIZE;

		cout << n_blocks << " blocks" << endl;

		if (ptr_in_Frame->isContinuous())
		{
			cout << "is Continuous" << endl;
		}

		//char* data_start = (char*)(ptr_in_Frame->data);

		char * longbuf = new char[SOCKET_BLOCK_SIZE * n_blocks];

		int64 start = getTickCount();

		cout << "start transfer \t" << hex << int(longbuf) << dec << endl;

		while (n < n_blocks)
		{
			//cout << n << "\r";
			sock.recvfrom(longbuf + n * SOCKET_BLOCK_SIZE, SOCKET_BLOCK_SIZE, &ep);

			sock.send("ready", 6);
			n++;
		}

		cout << "end transfer " << n << " blocks " << 
			(getTickCount() - start)/ getTickFrequency() << " s" << endl;

		

		Mat rawData = Mat(1, 65536, CV_8UC1, longbuf);

		Mat frame = imdecode(rawData, CV_LOAD_IMAGE_COLOR);

		if (frame.size().width == 0) {
			cerr << "decode failure!" << endl;
			continue;
		}

		imshow("recv", frame);
	
		//imshow("LK", *ptr_in_Frame);
		waitKey(100);

//		if (PC.proceed_frame(ptr_in_Frame)) break;

		//TODO skip frames

    }

	sock.close();

    
}
