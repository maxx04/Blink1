#pragma once
#include "xsocket.hpp"
#include <iostream>
#include <thread> 

class UDP_Base
{
public:
	std::thread* th1;


	UDP_Base();
	~UDP_Base();

	static void start();
	//void* ThreadSearch(void* args)
};

