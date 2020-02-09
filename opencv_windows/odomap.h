#pragma once
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

// Struktur zu beschreibung von jedem Schritt des Bewegung
struct diskret_move
{
	//Vektor fuer die Bewegung beim Schritt (Frame oder n Frames)
	//HACK Aktuell kamera ist fest zu Koerper
	Point2f step;  
};

// Klasse zur Abbildung den Bewegungen aus odometryklasse in 2d
class odomap
{
public:
	const string map_window_name = "Map"; // Name für die Fenster die aktuelles Frame darstellt
	Mat map; // Abbildung fuer die Bewegung
	float scale = 1.0f;
	vector<diskret_move> trajektory; //volle Trajektorie

	odomap();
	
	void draw_map();
	void add_step(Point2f step);
};

