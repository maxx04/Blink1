#include "follower.h"



#define MOTOR_RECHTS_FW 23
#define MOTOR_RECHTS_BW 24
#define MOTOR_LINKS_FW 25
#define MOTOR_LINKS_BW 8

Point2f AimPoint;
bool setAimPt = false;

follower::follower()
{
	fneck.test();

	motor_r = new Motor(MOTOR_RECHTS_FW, MOTOR_RECHTS_BW);
	motor_l = new Motor(MOTOR_LINKS_FW, MOTOR_LINKS_BW);

	//motor_r -> test();
	//motor_l -> test();

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

void follower::swap()
{
	cv::swap(prevGray, gray);
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
	// TODO: Fügen Sie hier Ihren Implementierungscode ein..
	take_picture(frame);
/*
	if (needToInit)
	{
		find_keypoints();
		needToInit = false;
		kp.swap();
		return false;
	}

	calcOptFlow();

	check_for_followed_points();

//	check_for_followed_points(); //TODO zuerst finden die Punkte die gut sind (status) 
	//nur dann collect step vectors.

	collect_step_vectors(); //

	transform_Affine();

	calculate_move_vectors();

	int wait_time = draw_image();

	show_image();

	look_to_aim();

	if (key(wait_time)) return true;
	*/
	return false;
	
}

void follower::new_data_proceed(UDP_Base* udp_base)
{

	//fneck.read_udp_data(udp_base->udp_data->servo_position_x,
	//	udp_base->udp_data->servo_position_y);


	//cout << "new servo-position: " << p << endl;

	fneck.move_to(udp_base->udp_data->angle_horizontal,
		udp_base->udp_data->angle_vertikal);

	udp_base->udp_data_received();

	//send antwort an client 
}


