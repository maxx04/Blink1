#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;

class histogram
{
private:
	Mat plotResult;
	int window_width;
	int windows_high;
	int windows_h_offset; //base Histogramm


public:
	int dims;
	float range_max;
	float range_min;
	int bins;
	vector<float> values;
	vector<int> bins_counters;
	vector<float> bins_borders;
	int values_index;

	histogram();
	histogram(int bins, int dims = 1);
	~histogram();

	void plot_result(Point p);
	
	int collect(float value);
	int calculate();
	void clear();


};

