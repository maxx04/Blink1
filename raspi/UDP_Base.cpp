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
bool new_udp_data = false;

static union_data dt;

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
	v6s.recvfrom(dt.buf512, 512, &ep);
	std::cout << "erstes pack, buffer: " << buff << std::endl;
	std::cout << ep.to_string() << std::endl;

	while (true) 
	{
		int i = v6s.recvfrom(dt.buf512, 512, &ep);

		cout << "i: " << i << endl;
		if (buff == "qiut" || i == -1)	break; //TODO quit bedingungen korrigieren

		new_udp_data = true;

		std::cout << "packet from: " << ep.to_string() << std::endl
			<< "DATA START" << std::endl <<
			dt.data.servo_position
			<< std::endl
			<< "DATA END" << std::endl;

	//		std::string msg = ep.get_ip();

		dt.data.servo_position += 1.5;

	//TODO wenn gibtes neues antwort dann senden
			v6s.sendto(dt.buf512,512,ep); //TODO aendern auf UDP_BLOCK_SIZE
		

	}

	v6s.close();
	cout << "closed" << endl;
}


UDP_Base::UDP_Base()
{
	// Driver Code 


		th1 = new thread(start_Server, 3);

		udp_data = &dt.data;

		new_data = ::new_udp_data;

		cout << "Thread started, Id: " << th1->get_id() << endl;	

		if (f == 1)
			cout << "Key element found" << endl;
		else
			cout << "Key not present" << endl;

}


UDP_Base::~UDP_Base()
{
	
}

void UDP_Base::udp_data_received()
{
	new_data = false;
	::new_udp_data = false;
}




