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




// Klass odometry ermittelt Bewegung des Robots und Abstand zu Schlüsselpunkten
class odometry
{
	const string main_window_name = "Frame"; // Name für die Fenster die aktuelles Frame darstellt
	int step_butch; // anzahl frames die auf block werden bearbeitet.
	// punktnummern in keypoints : vector POINTS die sind als Hintegrund angenommen
	vector<int> backround_points_numbers; 
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
	void check_for_followed_points(vector<Point2f>* prev_points, vector<Point2f>* current_points, 
		vector<uchar>* status, vector<float>* err);

	void find_followed_points();
	void draw_keypoints();
	void draw_background_points();
	void draw_main_points();
	void draw_summ_vector();
	int draw_image();
	void show_image();
	void kompensate_jitter();
	void kompensate_roll();
	void swap();
	bool key(int wait);
	void find_backround_points();
	void draw_flow();
	bool proceed_video(Mat* frame);
	void new_data_proceed(UDP_Base* udp_base);

};


