#include "UDP_Base.h"

using namespace std;

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

std::string buff;
net::endpoint ep;

// Linear search function which will 
	// run for all the threads 
void start_Server(int args)
{
	int num = current_thread++;

	for (int i = 0;	i < max; i++)
	{
		if (a[i] == key) f = 1;
	}

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
	v6s.recvfrom(&buff, 512, &ep);
	std::cout << "erstes pack, buffer: " << buff << std::endl;
	std::cout << ep.to_string() << std::endl;

	while (true) 
	{
		int i = v6s.recvfrom(&buff, 512, &ep);
		if (i == 4 || i == -1)
			break;
		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl << buff << std::endl
			<< "DATA END" << std::endl;

		//if (r > 0 && buff == "ip\n")
		{
			std::string msg = ep.get_ip();
			v6s.sendto(&msg,ep);
		}

	}

	v6s.close();
	cout << "closed" << endl;
}


UDP_Base::UDP_Base()
{
	// Driver Code 

		th1 = new thread(start_Server, 3);

		cout << "Thread started, Id: " << th1->get_id() << endl;

		

		if (f == 1)
			cout << "Key element found" << endl;
		else
			cout << "Key not present" << endl;

}


UDP_Base::~UDP_Base()
{
}



 void UDP_Base::start()
{


}
