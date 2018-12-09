#include "UDP_Base.h"

using namespace std;

static UDP_Base::dt;

UDP_Base::UDP_Base()
{
	// Driver Code 
//		assert(sizeof(exchange_data) > 512);

	

		//if (f == 1)
		//	cout << "Key element found" << endl;
		//else
		//	cout << "Key not present" << endl;

}


UDP_Base::~UDP_Base()
{
	
}

void UDP_Base::udp_data_received()
{
	new_data = false;
	new_udp_data = false;
}

void UDP_Base::start()
{
	th1 = new thread(start_Server, 3);

	// udp_data = &dt.dt_udp;

	//	new_data = new_udp_data;

	cout << "Thread started, Id: " << th1->get_id() << endl;
}




// Linear search function which will 
	// run for all the threads 
 
