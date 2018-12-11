#pragma once

#include <iostream>
#include <thread> 
#include <assert.h>

//#include <opencv2/core.hpp>

#include "xsocket.hpp"

//using namespace cv;

struct exchange_data 
{
	float servo_position_x;
	float servo_position_y;
};

union udata
{
	exchange_data dt_udp; 
	char buf512[512];
	
}; 


class UDP_Base
{
	std::thread* th1;

	static bool new_udp_data;

	static udata dt;

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

