#pragma once
#pragma warning(disable : 4996)

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <queue>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "Servos.h"
#include "camera_calibration.h"
#include "../UDP_Base.h"
#include "Motor.h"

using namespace cv;
using namespace std;

class follower
{
	//int step_butch; // anzahl frames die auf block werden bearbeitet.
	//TermCriteria termcrit;
	//Size subPixWinSize, winSize;
	
	float pixel_pro_step = 8.0; // TODO Übertragen in Servo?

	Point2f fokus;
	int number_aim_point = -1;
	double frame_time = 0.0;

	Servos s;
	Motor* motor_r;
	Motor* motor_l;

	//bool needToInit = false;
	//bool nightMode = false;

	Mat gray, prevGray, image;

	Mat Affine;
	Mat cameraMatrix;
	//keypoints kp; // keypoints von init_points()
	//histogram  hist; // histogramm für step_vectors in batch
	//Point2f main_of_frame; //ergebnissvektor fur frame verschiebung
	//float magnify_vektor_draw;

public:
	follower();
	~follower();
	//void find_keypoints();
	void take_picture(Mat* frame);
	//void check_for_followed_points();
	//void calcOptFlow();
	//void transform_Affine();
	//void draw_aim_point();
	//void draw_prev_points();
	//void draw_current_points();
	//void draw_calculated_points();
	//void draw_main_points();
	//void draw_summ_vector();
	//int draw_image();
	//void draw_step_vectors();
	//void draw_nearest_point();
	//void show_image();
	//void cam_calibrate();
	//void calculate_move_vectors();
	void swap();
	bool key(int wait);
	//void look_to_aim();
	//int find_nearest_point(Point2f pt);

	//int collect_step_vectors();
	// Bearbeitet jedes frame
	bool proceed_frame(Mat* frame);
	void new_data_proceed(UDP_Base* udp_base);
};


