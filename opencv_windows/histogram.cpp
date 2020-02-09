
#include "histogram.h"

histogram::histogram()
{
	values_index = 0;
	window_width = 300;
	windows_high = 120;
	mean = 0.0;
	main_mean = 0.0;
	windows_h_offset = 10; //base Histogramm
	plotResult.create(windows_high + windows_h_offset, window_width, CV_8UC3);
	plotResult.setTo(Scalar(0, 0, 0)); // hintergrund

	range_min = 1e10;
	range_max = -1e10;
}

histogram::histogram(int _bins, string _name, int _dims ) : histogram()
{
	bins = _bins;
	dims = _dims;
	name = _name;
	
	bins_group = new vector<point_satz>[bins];
}

int histogram::collect(point_satz value)
{

	if (value.v > range_max) range_max = value.v;
	if (value.v <= range_min) range_min = value.v;

	values.push_back(value);

	values_index++;

	mean = ((mean*(values_index - 1)) + value.v) / values_index;
		// ('previous mean' * '(count -1)') + 'new value') / 'count'
	
	return values_index;
}
		   
int histogram::sort(float rmn, float rmx)
{
	range_min = rmn;
	range_max = rmx;

	assert(bins < 300 && bins != 0);   // TODO begründen

	bins_borders.resize(bins + 1);
	bins_counters.resize(bins);

	bins_borders[0] = range_min;

	float step = (range_max - range_min) / (float)bins;

	//if (step == 0.0) return 0;

	for (int n = 0; n < bins+1; n++)
	{
		bins_borders[n] = step * (float)n + range_min;
	}


	for (point_satz p : values)
	{
		for (int n = 0; n < bins; n++)
		{
			if (p.v > bins_borders[n] && p.v <= bins_borders[n + 1])
			{
				bins_counters[n]++;
				bins_group[n].push_back(p);
				break;
			}
		}

	}

	main_mean = get_main_middle_value();

	plot_result(Point(0, 0));

	return 0;
}

int histogram::sort()
{
	return sort(range_min, range_max);
}

double histogram::get_main_middle_value()
{
	// Bedinnungen zu finden: max und daneben 70% punkten, mittelwert finden.
	// TODO verfinern kriterien
	int max_counter = 0;

	// Finden maximalen Strahl
	for (int n = 1; n < bins; n++)
	{
		if (bins_counters[max_counter] < bins_counters[n]) max_counter = n; // HACK n-1
	}

	int sum = 0; // Anzahl Treffer in 3 Strahlen
						 
	int max_bins[3]; // Nummer maximalen Strahl und Nachbarn

	if (max_counter == bins - 1)
	{
		max_bins[0] = max_counter - 1;
		max_bins[1] = max_counter;
		max_bins[2] = 0;
	}
	else if(max_counter == 0)
	{
		max_bins[0] = max_counter;
		max_bins[1] = 0;
		max_bins[2] = 1;
	}
	else
	{
		max_bins[0] = max_counter  - 1;
		max_bins[1] = max_counter;
		max_bins[2] = max_counter + 1;
	}

	sum = bins_group[max_bins[0]].size() 
		+ bins_group[max_bins[1]].size()
		+ bins_group[max_bins[2]].size();

	if ((float)sum/(float)values.size() < 0.1) //TODO Assert value index
	{
		return 0.0;
	}
	else
	{
		int n = 0;
		double result = 0.0;
		for (int i = 0; i < 3; i++)
		{
			for (point_satz s : bins_group[max_bins[i]]) 
			{
				result += s.v;
				// 
				//main_points->push_back(s.n);
				++n; //HACK 2 Grad + 358 Grad Mitte gibt 180 Grad
			}

		}

		double p = result / n;
		return p;
	}

	
}

void histogram::clear()
{
	values.clear();
	values_index = 0;
	mean = 0;
	main_mean = 0.0;
	bins_counters.clear();

	range_min = 1e10;
	range_max = -1e10;

	for (int i = 0; i < bins; i++)
	{
		bins_group[i].clear();
	}

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

	int max = 0; // Längste Strahl 

	for (int i = 0; i < bins; i++)
	{
		max = MAX(bins_counters[i], max);
	}

	//max = 200; //TODO auf anzahl den Punkten basierend.

	if (max == 0)
	{
		//TODO Assert hinzufügen
		cerr << "keine Werte fuer Histogramm" << endl;
		main_mean = 0.0;
		return;
	}

	int step = window_width / bins;
	int top = windows_high - windows_h_offset;

	assert(top != 0.0);

	float magnification = (float)max / (float)top; 
	
	//window Koordinaten oben_links

	//Zeichne Strahlen
	for (int i = 0; i < bins; i++)
	{
		int hi = (int)((float)(bins_counters[i]) / magnification); 
		rectangle(plotResult, Rect(i*step, top - hi, step, hi), Scalar(0,200,0),-5);
	}

	stringstream Titel;

	Titel << name << format(" %.1f/%.1f/%.1f", range_min, main_mean, range_max);

	setWindowTitle(name, Titel.str());
	
	imshow(name, plotResult);
	
}


