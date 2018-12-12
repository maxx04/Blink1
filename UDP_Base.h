#pragma once

#include <iostream>
#include <thread> 
#include <assert.h>

#include "xsocket.hpp"
#include <opencv2/core.hpp>

#define SOCKET_BLOCK_SIZE 512

using namespace cv;

struct exchange_data 
{
	float servo_position_x;
	float servo_position_y;
};


union udata
{
	exchange_data dt_udp; 
	char union_buff[SOCKET_BLOCK_SIZE];
	
}; 

union uFrame
{
	cv::Mat frm;
	char union_buff[SOCKET_BLOCK_SIZE];

};


class UDP_Base
{
	std::thread* udp_thread;

	static bool new_udp_data;

	static udata dt;
	static Mat* ptrFrame;

	static std::string buff;
	static net::endpoint ep;
	
public:

	exchange_data* udp_data;

	UDP_Base();
	~UDP_Base();
	void udp_data_received();
	bool check_incoming_data();
	static void start_Server(int args);

};

