#pragma once
#pragma warning(disable : 4996)



#include <iostream>
#include <string>
#include <ctime>
#include <queue>

#include "../UDP_Base.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "keypoints.h"
#include "histogram.h"


using namespace cv;
using namespace std;

#define MIN_FOLLOWED_POINTS 50


// Klass odometry ermittelt Bewegung des Robots und Abstand zu Schlüsselpunkten
class odometry
{
	const string main_window_name = "Frame"; // Name für die Fenster die aktuelles Frame darstellt
	int step_butch; // anzahl frames die auf block werden bearbeitet.

	Point2f fokus;	// Mittelpunkt vom Bild
	double frame_time = 0.0;
 	bool needToInitKeypoints = true;
	Mat gray, prevGray, image;
	//Mat Affine;
	Mat cameraMatrix;
	Mat distCoeffs;
	keypoints kp; // keypoints von 
	Point2f main_of_frame; // Ergebnissvektor fur 2d Bildverschiebung
	float magnify_vektor_draw;

public:
	odometry(Mat* frame);
	~odometry();
	void set_fokus(Mat* frame);
	void take_picture(Mat* frame);	// Bildvorbereitung
	void find_keypoints();

	void find_keypoints_FAST();

	// kontrolliert auf schlechte Punkte und loescht die
	// status - vector vom LukasKande
	// err -vector vom LukasKande
	void check_for_followed_points(vector<uchar>* status, vector<float>* err);

	void find_follow_points();
	//void transform_Affine();
	void draw_prev_points();
	void draw_keypoints();
	//	void draw_calculated_points();
	void draw_main_points();
	void draw_summ_vector();
	int draw_image();
	void draw_step_vectors();
	void draw_nearest_point();
	void show_image();
	//	void cam_calibrate();
	void kompensate_jitter(vector<int>* points_number);
	void kompensate_roll(vector<int>* points_number);
	void swap();
	bool key(int wait);
	int find_nearest_point(Point2f pt);
	int collect_step_vectors();
	void find_backround_points(vector<int>* backround_points_numbers);
	void draw_flow();
	bool proceed_video(Mat* frame);
	bool proceed_keypointsset(Mat* frame, std::vector <keypoints_flow>* key_points);  // Bearbeitet jedes frame
	void new_data_proceed(UDP_Base* udp_base);

};


