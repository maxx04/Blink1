
#include "histogram.h"


histogram::histogram()
{
	values_index = 0;
	//values = new vector<float>(1);
}


histogram::histogram(int _bins, int _dims ):histogram()
{
	bins = _bins;
	dims = _dims;
	range_min = 1e10;
	range_max = -1e10;
}

int histogram::collect(float value)
{
	if (value > range_max) range_max = value;
	if (value <= range_min) range_min = value;

	values[values_index++] = value;

	return values_index;
}

int histogram::calculate()
{
	float s = 0.0;

	for  (int i = 0; i < values_index; i++) //HACK nicht alle werte von values werden verwendet
	{
		s += values[i];
	}
	values_index = 0;

	return 0;
}

histogram::~histogram()
{
}
