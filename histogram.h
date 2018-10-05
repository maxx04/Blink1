#pragma once

#include <opencv2/core.hpp>
#include <vector>

using namespace std;

class histogram
{
private:
	


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
	~histogram();
	histogram(int bins, int dims = 1 );
	int collect(float value);
	int calculate();
	void clear();


};

