#include "UDP_Base.h"

using namespace std;

bool UDP_Base::new_udp_data = false;


udata UDP_Base::dt;

std::string UDP_Base::buff;
net::endpoint UDP_Base::ep;


UDP_Base::UDP_Base()
{

	cout << sizeof(exchange_data) << endl;

	assert(sizeof(exchange_data) < 512);

	th1 = new thread(start_Server, 3);

	udp_data = &dt.dt_udp;

	cout << "Thread started, Id: " << th1->get_id() << endl;

}


UDP_Base::~UDP_Base()
{

}

void UDP_Base::udp_data_received()
{
	new_udp_data = false;
}

bool UDP_Base::check_incoming_data()
{
	return new_udp_data;
}



void UDP_Base::start_Server(int args)
{


	//init only required for windows, no-op on *nix
	net::init();

	//create an ipv6 udp socket, we can optionaly specify the port to bind to as the 3rd arg
	net::socket v6s(net::af::inet, net::sock::dgram, 8080);

	if (!v6s.good()) {
		std::cerr << "failed to create & bind ipv4 socket" << std::endl;
		return;
	}

	std::cout << "ipv4 socket created..." << std::endl;

	//we can access a sockets local endpoint by getlocaladdr() && getremoteaddr() for tcp peers
	std::cout << "listening at: " << v6s.getlocaladdr().to_string() << std::endl
		<< "send a udp packet to :: " << v6s.getlocaladdr().get_port() << " to continue" << std::endl;

	//we can recv directly into a std::string or a char* buffer

	//recv a packet up to 512 bytes and store the sender in endpoint ep
	v6s.recvfrom(dt.buf512, 512, &ep);
	std::cout << "erstes pack, buffer: " << buff << std::endl;
	std::cout << ep.to_string() << std::endl;

	while (true)
	{
		int i = v6s.recvfrom(dt.buf512, 512, &ep);

		cout << "i: " << i << endl;
		if (i == -1)	break; //TODO quit bedingungen korrigieren

		new_udp_data = true;

		cout << "new_data_set " << new_udp_data << endl;

		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl <<
			dt.dt_udp.servo_position_x << ":" <<
			dt.dt_udp.servo_position_y
			<< std::endl
			<< "DATA END" << std::endl;

		//TODO wenn gibtes neues antwort dann senden
		v6s.sendto(dt.buf512, 512, ep); //TODO aendern auf UDP_BLOCK_SIZE


	}

	v6s.close();
	cout << "closed" << endl;
}


