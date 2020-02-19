#pragma once
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

extern class odometry;

using namespace cv;
using namespace std;


// Klasse zur Abbildung den Bewegungen aus odometryklasse in 2d
class odomap
{
public:
	const string map_window_name = "Map"; // Name für die Fenster die aktuelles Frame darstellt
	Mat map; // Abbildung fuer die Bewegung
	float scale = 1.0f;

	odomap();
	
	void draw_map(odometry* odm);

};

