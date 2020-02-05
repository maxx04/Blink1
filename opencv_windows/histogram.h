#pragma once



#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;


struct point_satz // Werte mit Nummern von Schluesselpunkten
{
	int n;	// Schlüsselpunktnummer
	float v; // Wert
};

// Allgemein Histogramm - Klass zu statistischen Auswertung
class histogram
{
private:
	Mat plotResult;	// Ergebnisabbildung
	int window_width;
	int windows_high;
	int windows_h_offset; // freie Abstand von Unten für die Histogramm


public:
	int dims; // Mehrdimensionaleshistogramm // TODO relisieren
	float range_max;  // maximales Wert
	float range_min;  // minimales Wert
	int bins; // Anzahl Segments
	String name; // Histogrambenennung
	vector<point_satz> values; // Werte
	vector<int> bins_counters; // Trefferanzahl für die Segmente
	vector<float> bins_borders;	// Segmentengrenzen
	vector<point_satz>* bins_group; // Einsortierte Werte in Segmenten
	int values_index; // Aktuelles fortlaufende Nummer beim Sammeln den Werten
	float mean;	// Mittelwert
	float main_mean; //wert wird berechnet in sort, meistens liegende werte 70% //TODO 

	histogram();

	// Konstruktor, 
	// bins - Anzahl Segmenten
	// name - Histogrammbenennung 
	histogram(int bins, string name, int dims = 1 );

	~histogram();  // Destruktor

	void plot_result(Point p);	// Histogramm zeichnen
	
	int collect(point_satz v);	// Werte sammeln

	int sort(float range_min, float range_max);	 // Werte einsortieren und Zeichnen	die mitgehen
	int sort();
	
	double get_main_middle_value();	// Histogramm-Mittelwert ausgeben

	void clear();  // Werte löschen

};

