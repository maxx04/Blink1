
#include "histogram.h"


histogram::histogram()
{
	values_index = 0;
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

	values.push_back(value);
	values_index++;

	return values_index;
}

int histogram::calculate()
{
	float s = 0.0;
	int max_counter = 0;

	bins_borders.resize(bins+1);
	bins_counters.resize(bins);

	bins_borders[0] = range_min;
	float step = (range_max - range_min) / (float)bins;

	for (int n = 0; n < bins; n++)
	{
		bins_counters[n] = 0;
		bins_borders[n] = step * (float)n + range_min;
	}

	for  (int i = 0; i < values_index; i++) //HACK nicht alle werte von values werden verwendet
	{
		for (int n = 0; n < bins; n++)
		{
			if (values[i] > bins_borders[n] && values[i] <= bins_borders[n + 1])
			{
				bins_counters[n]++;
			}
		}

	}

	for (int n = 1; n < bins; n++)
	{
		if (max_counter < bins_counters[n]) max_counter = n-1;
	}

	values_index = 0;

	clear();

	return 0;
}

void histogram::clear()
{
	values.clear();
}

histogram::~histogram()
{
}
