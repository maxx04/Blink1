#pragma once

#include <iostream>
#include <thread> 
#include "../xsocket.hpp"
#include <opencv2/core.hpp>

struct exchange_data {
	Point2f servo_position;
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



	UDP_Base();
	~UDP_Base();

	static void start();
	bool new_data;
};

