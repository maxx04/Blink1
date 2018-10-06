
#include "histogram.h"


histogram::histogram()
{
	values_index = 0;
	window_width = 300;
	windows_high = 120;
	windows_h_offset = 10; //base Histogramm
	namedWindow("Vectorhistogramm", 1);
	plotResult.create(windows_high + windows_h_offset, window_width, CV_8UC3);
	plotResult.setTo(Scalar(0, 0, 0)); // hintergrund

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

	plot_result(Point(0, 0));

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

void histogram::plot_result(Point p)
{
	plotResult.setTo(Scalar(0, 0, 0)); // hintergrund

	if (bins == 0)
	{
		//TODO Assert hinzufügen
		cerr << "keine bins in Histogramm" << endl;
		return;
	}

	int max = 0;

	for (int i = 0; i < bins; i++)
	{
		max = MAX(bins_counters[i], max);
	}

	if (max == 0)
	{
		//TODO Assert hinzufügen
		cerr << "maximale bin 0" << endl;
		return;
	}

	int step = window_width / bins;
	int top = windows_high - windows_h_offset;
	float magnification = (float)max / (float)top; //TODO assert
	
	//HACK window Koordinaten oben_links

	for (int i = 0; i < bins; i++)
	{
		int hi = (int)((float)(bins_counters[i]) / magnification); 
		rectangle(plotResult, Rect(i*step, top - hi, step, hi), Scalar(0,200,0),-5);
	}
	
	imshow("Vectorhistogramm", plotResult);
}


