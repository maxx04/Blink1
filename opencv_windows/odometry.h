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
#include "odomap.h"


using namespace cv;
using namespace std;


// Klass odometry ermittelt Bewegung des Robots und Abstand zu Schlüsselpunkten
class odometry
{
	const int min_background_points = 3;
	const int min_ground_points = 20;
	const int min_followed_points = 100;

	//friend class keypoints; //HACK 

	float focal_length;
	float VFOV2; // Vertikale Kameraansichtwinkel geteilt auf  [radian]
	float cam_v_distance; // Kameraabstand vom Boden [mm]
	float cam_pitch;  // Winkel zwischen Bodenebene und Horizotale Kameraebene [radian]
	float yaw_angle;  // roboter Azimut Winkel

	const string main_window_name = "Frame"; // Name für die Fenster die aktuelles Frame darstellt

	Point2f fokus;	// Mittelpunkt vom Bild	- "principial point"

	int frame_number = 0;
 	bool needToInitKeypoints = true;
	Mat gray, prevGray, image;

	//Mat Affine;
	Mat cameraMatrix;
	Mat distCoeffs;

	odomap map; // Bewegungsabbildung in 2d

	keypoints kp; // keypoints von 
	Point2f main_of_frame; // aktuelle Ergebnissvektor fur 2d Bildverschiebung
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


	void draw_keypoints();
	void draw_background_points();
	void draw_ground_points();
	//void draw_main_points();
	void draw_summ_vector();
	void draw_map();
	int draw_image();
	void draw_flow();

	void show_image();

	void kompensate_jitter();
	void kompensate_roll();

	void swap();
	bool key(int wait);

	float calc_step();
	void calc_distances(float step);  // berechnet distanz vom kamera zu punkten vor dem step Bewegung

	void find_background_points();
	void find_ground_points();
	void find_followed_points();

	bool proceed_video(Mat* frame);
	void find_yaw(float step);
	void new_data_proceed(UDP_Base* udp_base);

	

};


