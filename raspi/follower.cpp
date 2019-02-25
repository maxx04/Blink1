#include "follower.h"


Point2f AimPoint;
bool setAimPt = false;

follower::follower()
{
//	fneck.test();

	termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
	subPixWinSize = Size(6, 6);
	winSize = Size(11, 11);

/*
	FileStorage ks("out_camera_data.xml", FileStorage::READ); // Read the settings
	if (!ks.isOpened()) 
	{
		cout << " Camera matrix fixed" << endl;
		float data[10] = { 700, 0, 320, 0, 700, 240, 0, 0, 1 };
		cameraMatrix = Mat(3, 3, CV_32FC1, data); // rows, cols

		float data1[5] = { -4.1802327018241026e-001, 5.0715243805833121e-001, 0., 0.,
					-5.7843596847939704e-001 };

		distCoeffs = Mat(5, 1, CV_32FC1, data1); // rows, cols
	}
	else
	{
		cout << "Camera matrix loaded" << endl;
		ks["Camera_Matrix"] >> cameraMatrix;
		ks["Distortion_Coefficients"] >> distCoeffs; //TODO
	}
*/
	tm.reset();
}

follower::~follower()
{
}

// Bearbeitet Frame in schritten
bool follower::proceed_frame(Mat* frame, int frame_index)
{

	take_picture(frame);

	if (needToInit)
	{
		// nur erstes frame wird ausgeführt
		cout << "find keypoints: " << endl;

		kpt.clear();
		kpt_diff.clear();

		find_keypoints();

		needToInit = false;

		// nicht weiter bearbeiten
		return false;
	}

	if (prev_image.size().width != 0)
	{
		tm.reset();
		tm.start();

		calcOptFlow(frame_index);

		tm.stop();

		cout << "optical Flow compute " << tm << " Keypoints: " << kpt.size() << endl;
	}


	if (key(wait_time)) return true;

	return false;

}


void follower::take_picture(Mat* frame)
{
	if (frame->empty())
	{
		cerr << " Frame empty \n";
		return; //TODO Fehlerabarbeitung
	}

	image.copyTo(prev_image);

	cvtColor(*frame, image, COLOR_BGR2GRAY);

}

void follower::find_keypoints()
{
	
	//finde features
	tm.reset();
	tm.start();

	kpt.clear();

	goodFeaturesToTrack(image, kpt, 500, 0.03, 10, Mat(), 9, 5);

	//refine position
	cornerSubPix(image, kpt, subPixWinSize, Size(-1, -1), termcrit);

	tm.stop();

	cout << "Features compute " << tm << " kp " << kpt.size() << endl;
	
}


bool follower::key(int wait)
{
	char c = (char)waitKey(wait);

	if (c == 27) return true;

	switch (c)
	{
	case 'r':
		break;

	case 'c':
		break;

		case 'k':
		//	cam_calibrate(&cameraMatrix, &distCoeffs);
			break;
	}

	return false;
}



void follower::find_diff_keypoints()
{



}

void follower::calcOptFlow(int frame_index)
{
//	Point2f p;

	if (kpt.size() != 0)
	{
		// kpt muss nicht leer sein
		kpt.swap(prev_kpt);

		calcOpticalFlowPyrLK(prev_image, image, /*prev*/ prev_kpt, /*next*/ kpt,
			status, err, winSize, 5, termcrit, 0, 0.001);

		if (kpt_diff.size() == 0) kpt_diff.resize(kpt.size()); // OPTI immer pruefen?

		if (frame_index != 0) //wenn nicht erste frame
		{
			for (size_t i = 0; i < kpt.size(); i++)
			{
				//bei mehrerer aufnahmen summieren flow
				kpt_diff[i] += kpt[i] - prev_kpt[i];
			}
		}
		else
			cerr << "OptFlow mit erstem Bild" << endl;

		clean_bad_keypoints();
	}

}

void follower::clean_bad_keypoints()
{
	for (size_t i = 0; i < kpt.size(); i++)
	{
		if (status[i] != 1 && err[i] < 10.0)
		{
			kpt.erase(kpt.begin() + i);
			kpt_diff.erase(kpt_diff.begin() + i); //OPTI erase zu langsam
		}
	}

	cout << "new size kpt: " << kpt.size() << endl;
}

void follower::copy_keypoints()
{
	UDP_Base::key_points.clear();

	for (size_t i = 0; i < kpt.size(); i++) //TODO und weniger als 300
	{
		UDP_Base::key_points.push_back({ kpt[i], kpt_diff[i] });
	}

}

void follower::new_data_proceed(UDP_Base* udp_base)
{

	fneck.move_to(udp_base->udp_data->angle_horizontal,
		udp_base->udp_data->angle_vertikal);

	// rukmeldung alles in ordnung
	udp_base->udp_data_received();

	//send antwort an client 
}

void follower::start_move(UDP_Base* udp_base) 
{
	fdriver.start_move( udp_base->udp_data->stright_velocity);
}
void follower::stop_move(UDP_Base* udp_base)
{
	fdriver.stop_move(udp_base->udp_data->stright_velocity);
}




