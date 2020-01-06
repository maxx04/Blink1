#include <iostream>
#include <string>
#include "odometry.h"
#include "../UDP_Base.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

using namespace std;
using namespace cv;

int receive_frame(net::socket& sock, net::endpoint& ep, cv::Mat& frame);
void proceed_video(std::string& input, odometry& PC);
int proceed_udp(odometry& PC, bool& retflag);
int receive_keypoints(net::socket& sock, net::endpoint& ep);

std::vector <keypoints_flow> key_points(500);

std::string robi_ip = "192.168.178.20";

static void help()
{
	// print a welcome message, and the OpenCV version
	std::cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	std::cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	std::cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		<< endl;
}

int main(int argc, char** argv)
{
	help();
	cv::CommandLineParser parser(argc, argv, "{@input|0|}");
	string input = parser.get<string>("@input");

	odometry PC;

	if (input.size() == 1)
	{
		bool retflag;
		int retval = proceed_udp(PC, retflag);
		if (retflag) return retval;
	}
	else
	{
		proceed_video(input, PC);
	}

	//	sock.close();

}
// Funktion empfängt letztes Bild und "keypints flow" von UDP server (roboter) und bearbeitet die 
// @retflag	- true wenn program soll beendet werden
int proceed_udp(odometry& PC, bool& retflag)
{
	//retflag = true;
	Mat  frame;

	net::endpoint ep(robi_ip, 4010);

	//we must call net::init() on windows, if not on windows it is a no-op
	net::init();

	//create a socket without binding in the ctor
	net::socket sock(net::af::inet, net::sock::dgram, 4010);

	//ep = net::endpoint(robi_ip, 4010);

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
		_data.dt_udp.angle_horizontal = 0.0f;
		_data.dt_udp.angle_vertikal = 10.0f;
		_data.dt_udp.move_stright = 600.0f;
		_data.dt_udp.stright_velocity = 0.3f;
		_data.dt_udp.direction = 0.0f;

		// sende Bewegungsdaten für roboter
		sock.send(_data.union_buff, SOCKET_BLOCK_SIZE);

		if (receive_frame(sock, ep, frame) == 2) break;

		if (receive_keypoints(sock, ep) == 2) break;

		if (frame.size().width == 0)
		{
			cerr << "decode failure!" << endl;
			continue;
		}

		if (PC.proceed_keypointsset(&frame, &key_points)) break;

	}
	//retflag = false;
	return 0;
}

void proceed_video(std::string& input, odometry& PC)
{
	Mat  vframe;

	VideoCapture cap(input);

	if (!cap.isOpened())
	{
		std::cout << " Datei " << input << " nicht geoeffnet" << endl;
		exit(4);
	}


	while (true)
	{
		cap.read(vframe);

		if (vframe.rows == 0) break;  //OPTI mehr proffesionel machen

		if (PC.proceed_video(&vframe)) break;

		// dann zurueck zu frames
	}
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

	key_points.clear(); // OPTI merfaches kopieren vermeiden

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
