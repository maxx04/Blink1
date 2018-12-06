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

	//create a udp inetV4 socket and bind it to port 8080
	net::socket sock(net::af::inet, net::sock::dgram, 8080);

	//make sure socket creation and binding did not fail
	if (!sock.good()) {
		std::cerr << "error creating socket" << std::endl;
		return;
	}

	std::cout << "listening on port: " << sock.getlocaladdr().get_port() << std::endl;

	net::endpoint remoteAddr;
	std::string buf;

	while (true) {

		//accept() returns the fd which you can directly copy into a net::socket
		int r = sock.recvfrom(&buf, 1024, &remoteAddr);

		if (r > 0 && buf == "ip\n") {
			std::string msg = remoteAddr.get_ip();
			sock.sendto(&msg, remoteAddr);
		}
	}


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
