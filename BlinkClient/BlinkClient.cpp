// BlinkClient.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//

#include "pch.h"
#include <iostream>
#include "../xsocket.hpp"

int main()
{
	int i;

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

	std::string send_str = "ip\n";

	sock.send(&send_str); 
	
	Sleep(1000);

	sock.send(&send_str);

	while (true) 
	{
		sock.send(&send_str);

		int i = sock.recvfrom(&buff, 512, &ep);

		if (i == 4 || i == -1)	break;

		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl << buff << std::endl
			<< "DATA END" << std::endl;

		std::cin >> send_str;
	}

	sock.close();

}
