#pragma once

#include <iostream>
#include <thread> 
#include <assert.h>

#include "xsocket.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

#define SOCKET_BLOCK_SIZE 4096

using namespace std;

struct exchange_data 
{
	float angle_horizontal;
	float angle_vertikal;
	float move_stright;
	float stright_velocity;
	float direction;
};


union udata
{
	exchange_data dt_udp; 
	char union_buff[SOCKET_BLOCK_SIZE];
	
}; 

union int_char
{
	int nb;
	char bf[sizeof(int)];
};


class UDP_Base
{
	std::thread* udp_thread;
	static bool new_udp_data;
	static udata dt;
	static net::endpoint ep;
	
public:

	static bool transfer_busy;
	static bool imagegrab_ready;
	static std::vector <uchar> encoded;
	exchange_data* udp_data;

	UDP_Base();
	~UDP_Base();
	void udp_data_received();
	bool check_incoming_data();
	static void start_Server(int args);

};

