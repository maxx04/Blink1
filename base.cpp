#include <iostream>
#include <string>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif


#include "station.h"
#include "UDP_Base.h"

int receive_frame(net::socket& sock, net::endpoint& ep, cv::Mat& frame);
int receive_keypoints(net::socket& sock, net::endpoint& ep);

std::vector <keypoints_flow> key_points(500);

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
		<< endl;
}



int main(int argc, char** argv)
{

	int wait_time = 0;

	help();
	cv::CommandLineParser parser(argc, argv, "{@input|0|}");
	string input = parser.get<string>("@input");

	Mat  frame;

	net::endpoint ep;

	station PC;

	if (input == "")
	{
		VideoCapture cap;

		//we must call net::init() on windows, if not on windows it is a no-op
		net::init();

		//create a socket without binding in the ctor
		net::socket sock(net::af::inet, net::sock::dgram, 4010);

		ep = net::endpoint("192.168.178.47", 8080);

		//most calls in xsocket return the same value as there c counterparts
		//like so if sock.bind returns -1 it failed
		int r = sock.connect(ep);
		if (r == -1)
		{
			perror("failed to connect");
			return -1;
		}

		std::cout << "socket bound to: " << sock.getlocaladdr().to_string() << std::endl;

		// sock.send("connect", 8);

		Sleep(1000);

		udata _data;

		for (;;)
		{
			//if (PC.n .kp.needToInit)
			//{
			//	_data.dt_udp.angle_horizontal = 0.0f;
			//	_data.dt_udp.angle_vertikal = 0.0f;
			//	_data.dt_udp.move_stright = 0.0f;
			//	_data.dt_udp.stright_velocity = 0.0f;
			//	_data.dt_udp.direction = 0.0f;
			//}
			//else
			//{ 
			_data.dt_udp.angle_horizontal = 0.0f;
			_data.dt_udp.angle_vertikal = 10.0f;
			_data.dt_udp.move_stright = 600.0f;
			_data.dt_udp.stright_velocity = 0.3f;
			_data.dt_udp.direction = 0.0f;
			//}
			//cout << " h:"; cin >> _data.dt_udp.angle_horizontal;
			//cout << " v:"; cin >> _data.dt_udp.angle_vertikal;
			//cout << " dist:"; cin >> _data.dt_udp.move_stright;
			//cout << " dir:"; cin >> _data.dt_udp.direction;

			sock.send(_data.union_buff, SOCKET_BLOCK_SIZE);


			if (receive_frame(sock, ep, frame) == 2) break;
			if (receive_keypoints(sock, ep) == 2) break;


			if (frame.size().width == 0)
			{
				cerr << "decode failure!" << endl;
				continue;
			}

			if (PC.proceed_frame(&frame, &key_points)) break;

		}
	}
	else
	{
		VideoCapture cap(input);

		if (!cap.isOpened())
		{
			cout << " Datei " << input << " nicht geoeffnet" << endl;
			exit(4);
		}

		bool needToInit = true;
		vector<Point2f> kpt, prev_kpt;
		vector<Point2f> kpt_diff;
		TermCriteria termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
		Size subPixWinSize = Size(9, 9);
		Size winSize = Size(17, 17);
		vector<uchar> status; // status vom calcOpticalFlowPyrLK
		vector<float> err; // error vom calcOpticalFlowPyrLK

		Mat gray, prev_image;

		while (true)
		{
			cap.read(frame);
			// erstelle vector <keypoints_flow>

				// nur erstes frame wird ausgeführt
				cout << "find keypoints: " << endl;

				kpt.clear();
				kpt_diff.clear();

				cvtColor(frame, gray, COLOR_BGR2GRAY);

				//finde features
				goodFeaturesToTrack(gray, kpt, 500, 0.03, 10, Mat(), 9, 5);

				//refine position
				cornerSubPix(gray, kpt, subPixWinSize, Size(-1, -1), termcrit);

				key_points.resize(kpt.size());

				for (size_t i = 0; i < kpt.size(); i++) //TODO und weniger als 300
				{

					key_points[i].p = kpt[i];
					//key_points[i].flow[0] = kpt_diff[i];
				}

				//keypoints_number = kpt.size();

				needToInit = false;
				gray.copyTo(prev_image);


			for (int n = 0; n < ANZAHL_AUFNAHMEN; n++)  // 5 mal frame kopieren
			{
				cap.read(frame);
				// erstelle vector <keypoints_flow>
				cvtColor(frame, gray, COLOR_BGR2GRAY);

				if (prev_image.size().width != 0)
				{
					cout << "optical Flow compute... ";

					if (kpt.size() != 0)
					{
						// kpt muss nicht leer sein
						kpt.swap(prev_kpt);

						calcOpticalFlowPyrLK(prev_image, gray, /*prev*/ prev_kpt, /*next*/ kpt,
							status, err, winSize, 5, termcrit, 0, 0.001);

						if (kpt_diff.size() == 0) kpt_diff.resize(kpt.size()); // OPTI immer pruefen?
 
						for (size_t i = 0; i < kpt.size(); i++)
						{
							Point2f tmp = kpt[i] - prev_kpt[i];
							//bei mehrerer aufnahmen summieren flow
							kpt_diff[i] += tmp;
							// kopiere auch getrennt
							key_points[i].flow[n] = tmp;
						}

 						//bereinigung kp

						cout << "size kpt: " << kpt.size() << endl;

						for (size_t i = 0; i < kpt.size(); i++)
						{
							if (status[i] != 1 && err[i] < 10.0)
							{
								kpt.erase(kpt.begin() + i);
								kpt_diff.erase(kpt_diff.begin() + i); //OPTI erase zu langsam
								// bereinige auch zwischenbilder (vollständiges punkt)
								key_points.erase(key_points.begin() + i);
							}
						}

						cout << "new size kpt: " << kpt.size() << endl;
					}


					cout << " Keypoints: " << kpt.size() << endl;
				}
			}

			key_points.resize(kpt.size());

			if (PC.proceed_frame(&frame, &key_points)) break;

			// dann zurueck zu frames
		}
	}
	//	sock.close();


}

int receive_frame(net::socket& sock, net::endpoint& ep, cv::Mat& frame)
{

	int_char tmp;

	int i = sock.recvfrom(tmp.bf, sizeof(int), &ep); // 1. antwort

	if (i == -1)  return 2;

	std::cout << "packet from: " << ep.to_string() << std::endl;

	//Bild empfangen

	int n = 0;

	size_t n_blocks = tmp.nb;

	std::cout << n_blocks << " blocks" << endl;

	char* longbuf = new char[SOCKET_BLOCK_SIZE * n_blocks];

	int64 start = getTickCount();

	std::cout << "start transfer \t" << hex << int(&longbuf) << dec << endl;

	while (n < n_blocks)
	{
		//cout << n << "\r";
		sock.recvfrom(longbuf + n * SOCKET_BLOCK_SIZE, SOCKET_BLOCK_SIZE, &ep);

		sock.send(tmp.bf, sizeof(int));
		n++;
	}

	std::cout << "end transfer frame" << n << " blocks " <<
		(getTickCount() - start) / getTickFrequency() << " s" << endl;


	Mat rawData = Mat(1, n * SOCKET_BLOCK_SIZE, CV_8UC1, longbuf);

	frame = imdecode(rawData, IMREAD_COLOR);

	return 1;
}

int receive_keypoints(net::socket& sock, net::endpoint& ep)
{

	int_char tmp;

	int i = sock.recvfrom(tmp.bf, sizeof(int), &ep); // 1. antwort

	if (i == -1) { return 2; };

	std::cout << "packet from: " << ep.to_string() << std::endl;

	//Bild empfangen

	int points_nmb = tmp.nb;

	cout << "points : " << points_nmb << endl;

	int n_blocks = 1 + ((points_nmb * sizeof(keypoints_flow) - 1) / SOCKET_BLOCK_SIZE);

	std::cout << n_blocks << " blocks" << endl;

	char* longbuf = new char[SOCKET_BLOCK_SIZE * n_blocks];

	int64 start = getTickCount();

	std::cout << "start transfer keypoints \t" << hex << int(&longbuf) << dec << endl;

	int n = 0;

	while (n < n_blocks)
	{
		cout << n << "\n";
		sock.recvfrom(longbuf + n * SOCKET_BLOCK_SIZE, SOCKET_BLOCK_SIZE, &ep);

		sock.send(tmp.bf, sizeof(int));
		n++;
	}

	std::cout << "end transfer keypoints" << n << " blocks " <<
		(getTickCount() - start) / getTickFrequency() << " s" << endl;

	keypoints_flow* p;

	p = reinterpret_cast<keypoints_flow*>(longbuf);

	key_points.clear(); // OPTI merfahes kopieren vermeiden

	cout.fixed;

	for (size_t i = 0; i < points_nmb; i++)
	{
		//cout.precision(6);
		//cout.width(6);
		//cout << p[i].p << " - ";
		//
		////cout.precision(3);
		//cout.width(4);
		//for (size_t n = 0; n < ANZAHL_AUFNAHMEN; n++)
		//{
		//	cout << p[i].flow[n] << ":";
		//}
		//
		//cout << endl;

		//TODO alle zwischenpunkten aufnehmen

		key_points.push_back(p[i]); //TODO gibt es negative Punkte! vorher bereinigen
	}

}
