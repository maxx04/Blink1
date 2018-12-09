// BlinkClient.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//

#include "pch.h"
#include <iostream>
#include "../xsocket.hpp"
#include "../raspi/UDP_Base.h"

int main()
{
	int i;

	union_data _data;

	std::string buff;
	net::endpoint ep;

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

	sock.send(_data.buf512, 512); 
	
	Sleep(1000);

	while (true) 
	{
		float inp;

		std::cin >> inp;
		std::cout << inp << std::endl;

		_data.data.servo_position.x = inp;

		sock.send(_data.buf512, 512);

		int i = sock.recvfrom(_data.buf512, 512, &ep);

		if (i == -1)	break;

		std::cout << "antwort position : " << _data.data.servo_position.x << std::endl;

		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl <<
			buff 
			<< std::endl
			<< "DATA END" << std::endl;

		
	}

	sock.close();

}
