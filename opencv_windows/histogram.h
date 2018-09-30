#pragma once

#include <opencv2/core.hpp>
#include <vector>

using namespace std;

class histogram
{
public:
	int dims;
	float range_max;
	float range_min;
	int bins;
	vector<float> values;
	int values_index;

	histogram();
	~histogram();

	histogram(int bins, int dims = 1 );
	int collect(float value);
	int calculate();

};

