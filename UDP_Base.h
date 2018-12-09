#pragma once

#include <iostream>
#include <thread> 

#include <opencv2/core.hpp>

#include "xsocket.hpp"

using namespace cv;


struct exchange_data {
	Point2f servo_position;
};

union union_data
{
	exchange_data dt_udp; 
	char buf512[512];
	
}; 


class UDP_Base
{
	std::thread* th1;
	// Max size of array 
#define max 16 

// Max number of threads to create 
#define thread_max 4 

	int a[max] = { 1, 5, 7, 10, 12, 14, 15,
				   18, 20, 22, 25, 27, 30,
				   64, 110, 220 };
	int key = 220;

	// Flag to indicate if key is found in a[] 
	// or not. 
	int f = 0;

	int current_thread = 0;
	bool new_udp_data = false;

	union_data dt;

	std::string buff;
	net::endpoint ep;
	
public:

	exchange_data* udp_data;
	bool new_data;


	UDP_Base();
	~UDP_Base();
	void udp_data_received();
	void start_Server(int args);

};

