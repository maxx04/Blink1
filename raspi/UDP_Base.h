#pragma once

#include <iostream>
#include <thread> 
#include "../xsocket.hpp"
//#include <opencv2/core.hpp>

struct exchange_data {
	float servo_position;
};

union union_data
{
	exchange_data data; //TODO sizeof pruefen mit acc
	char buf512[512];
}; 


class UDP_Base
{
	std::thread* th1;
	
public:

	exchange_data* udp_data;

	UDP_Base();
	~UDP_Base();
	void udp_data_received();

	bool new_data;
};

