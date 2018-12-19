#include "follower.h"


Point2f AimPoint;
bool setAimPt = false;

follower::follower()
{
	fneck.test();

	float data[10] = { 700, 0, 320, 0, 700, 240, 0, 0, 1 };

	cameraMatrix = Mat(3, 3, CV_32FC1, data); // rows, cols
}

follower::~follower()
{
}



void follower::take_picture(Mat* frame)
{
	if (frame->empty())
	{
		cerr << " Frame empty \n";
		return; //TODO Fehlerabarbeitung
	}

}


bool follower::key(int wait)
{
	char c = (char)waitKey(wait);

	if (c == 27) return true;

	switch (c)
	{
	case 'r':
		//needToInit = true;
		break;

	case 'c':
		break;

		//case 'k':
		//	cam_calibrate();
	}

	return false;
}

// Bearbeitet Frame in schritten
bool follower::proceed_frame(Mat* frame)
{
	take_picture(frame);

	if (key(wait_time)) return true;

	return false;
	
}

void follower::new_data_proceed(UDP_Base* udp_base)
{



	fneck.move_to(udp_base->udp_data->angle_horizontal,
		udp_base->udp_data->angle_vertikal);

	fdriver.move(udp_base->udp_data->move_stright);

	fdriver.change_direction(udp_base->udp_data->direction);

	udp_base->udp_data_received();

	//send antwort an client 
}


