#include "follower.h"

Point2f AimPoint;
bool setAimPt = false;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		AimPoint = Point2f((float)x, (float)y);
		setAimPt = true;
		
	}
}

follower::follower()
{
	termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	subPixWinSize = Size(10, 10);
	winSize = Size(31, 31);

	needToInit = true;



//#ifndef _ARM
	namedWindow("LK Demo", 1);

	setMouseCallback("LK Demo", onMouse, 0);
//#endif
}


follower::~follower()
{
}

void follower::init_points()
{
	if (needToInit)
	{
		// automatic initialization
		points[0].clear();
		points[1].clear();
		//finde features
		goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);

		//resfine position
		cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);



		needToInit = false;
	}
}

void follower::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	fokus.x = (float)(image.cols / 2);
	fokus.y = (float)(image.rows / 2); //TODO nur einmal machen
	swap();
	frame -> copyTo(image);
	cvtColor(image, gray, COLOR_BGR2GRAY);
}

void follower::calcOptFlow()
{
	vector<uchar> status;
	vector<float> err;

	if (!points[0].empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray);

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ points[0], /*next*/ points[1],
			status, err, winSize, 5, termcrit, 0, 0.001);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		Affine = estimateRigidTransform(points[0], points[1], true);

		 //needToInit = true;
	}
}

void follower::transform_Affine()
{
	if (!Affine.empty() && !points[0].empty())
	{
		//cout << points[0].size() << " - " << calc[0].size() << endl;

		// Affine_x_last = Affine_x;

		// Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
											// umrechnen feautures
		transform(points[0], calc[0], Affine);

	}
}

void follower::draw()
{
	calc[1].resize(points[1].size());

	for (size_t i = 0; i <  points[0].size(); i++)
	{
		// draw berechnete features
		circle(image, (Point)points[0][i], 4, Scalar(255, 0, 0));
	}

	//for (size_t i = 0; i < points[1].size(); i++)
	//{
	//	// draw berechnete features
	//	circle(image, (Point)points[1][i], 4, Scalar(255, 0, 255));
	//}

	for (size_t i = 0; i < min(points[1].size(), points[0].size()); i++)
	{
		Point2f p0 = points[0][i];
		Point2f p1 = points[1][i];

		line(image, (Point)p0, (Point)p1, Scalar(255, 100, 0));

		//calc[1][i] = points[1][i] - points[0][i];
	}
}

void follower::show()
{
	stringstream text;

	//text.width(4);
	text.precision(2);

	//text << "calc " << timeSec * 1000 << " ms " << "  " << points[1].size() << "  " <<  Affine_x * 10;

	text << "calc " << points[0].size() << " | " << points[1].size();

	//putText(image, text.str(), Point(100, 100), FONT_HERSHEY_PLAIN, 2.0f, Scalar(0, 0, 0), 2);

//#ifndef _ARM

	setWindowTitle("LK Demo", text.str());

	imshow("LK Demo", image);

//#endif
}

void follower::swap()
{
	std::swap(points[1], points[0]);
	cv::swap(prevGray, gray);
}

bool follower::key()
{
	char c = (char)waitKey(10);

	if (c == 27) return true;

	switch (c)
	{
	case 'r':
		needToInit = true;
		break;

	case 'c':
		points[0].clear();
		points[1].clear();
		break;
	}

	return false;
}

void follower::look_to_aim(Servos* s)
{
	float richtung = 0.0;
	Point2f m;

	if (setAimPt)
	{
		// 1) finde positiondifferenz
		number_aim_point = find_nearest_point(AimPoint);
		setAimPt = false;
	}

	if (number_aim_point < 0) return;

	// 1) finde positiondifferenz
	m = points[1][number_aim_point] - fokus;
	// 2) finde richtung
	richtung = (m.x < 0) ? 10.0 : -10.0;
	// 3a) finde wo ist jetzt den Punkt (z.B. über Matrix)
	// kontrolle über vergleich p[0] - P[1]
	// 4) wenn differenz immer noch groß, zu gehe zu schritt 1. 
	// 3) bewege einen schritt in Richtung
	if (abs(m.x) > 15.0)
	{
		s->correction(richtung);
	}
	else
	{
		number_aim_point = -1;
		needToInit = true;
	}

}

int follower::find_nearest_point(Point2f pt)
{
	float dist = 100000.0;
	int n = 0; //TODO wenn 0 bearbeiten
	for (size_t i = 0; i < points[1].size(); i++)
	{
		// draw berechnete features
		if (abs(AimPoint.x - points[1][i].x) < dist)
		{
			dist = abs(AimPoint.x - points[1][i].x);
			n = i;
		}

	}
	return n;
}
