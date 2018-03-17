#include "follower.h"

Point2f AimPoint;
bool setAimPt = false;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		AimPoint = Point2f((float)x, (float)(y)); 
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
		goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.05, 12, Mat(), 5, 5, 0, 0.04);

		//refine position
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


	if (!points[0].empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray);

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ points[0], /*next*/ points[1],
			status, err, winSize, 3, termcrit, 0, 0.001);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		Affine = estimateRigidTransform(points[0], points[1], true);

		 //needToInit = true;
	}
}

void follower::transform_Affine()
{
	if (!Affine.empty() && !points[0].empty())
	{
		// Affine_x_last = Affine_x;

		//Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
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
		if (status[i] == 1)
			circle(image, (Point)points[0][i], 4, Scalar(255, 0, 255));
		else
			circle(image, (Point)points[0][i], 4, Scalar(255, 0, 0));
	}

	if(number_aim_point >= 0 )
	circle(image, (Point)points[1][number_aim_point], 16, Scalar(0, 0, 255), 3);

	circle(image, (Point)AimPoint, 16, Scalar(0, 255, 0), 3);

	double Affine_x = 0.0;

	if (!Affine.empty() && !points[0].empty())
	{
		Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
	}
	for (size_t i = 0; i < min(points[1].size(), points[0].size()); i++)
	{
		Point2f p0 = points[0][i];
		Point2f p1 = points[1][i];

		//if (Affine_x != 0)
		//p1.x = ((p1.x - p0.x) / Affine_x) + p0.x;

		line(image, (Point)p0, (Point)p1, Scalar(255, 255, 100));

		//calc[1][i] = points[1][i] - points[0][i];
	}
}

void follower::show()
{
	stringstream text;

	text.width(5);
	text.precision(3);

	//text << "calc " << timeSec * 1000 << " ms " << "  " << points[1].size() << "  " <<  Affine_x * 10;

	int st = 0;

	for (size_t i = 0; i < status.size(); i++)
	{
		st += status[i];
	}

	text << "calc " << points[0].size() << " | " << points[1].size() << " | " << st;

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

void follower::look_to_aim()
{
	Point2f richtung = Point2f(0.0,0.0);
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
	richtung.x = -m.x / pixel_pro_step;
	richtung.y = m.y / pixel_pro_step;
	// 3a) finde wo ist jetzt den Punkt (z.B. �ber Matrix)
	// kontrolle �ber vergleich p[0] - P[1]
	// 4) wenn differenz immer noch gro�, gehe zu schritt 1. 
	// 3) bewege einen schritt in Richtung
	if (abs(m.x) > 20.0 || abs(m.y) > 20.0)
	{	

		s.correction(richtung);

		cout <<  points[1][number_aim_point] << m << richtung << "|" << s.position << endl;
	}
	else
	{
		cout << "find " << points[1][number_aim_point] << m << richtung << endl;
		number_aim_point = -1;
		needToInit = true;
	}



}

int follower::find_nearest_point(Point2f pt)
{
	float d, dist = 10000000.0;
	Point2f v;
	int n = 0; //TODO wenn 0 bearbeiten
	
	for (size_t i = 0; i < points[0].size(); i++)
	{
		// draw berechnete features
		v = points[0][i] - pt;
		d = v.x*v.x + v.y*v.y;
		if (d < dist && status[i] == 1)
		{
			dist = d;
			n = i;
		}

	}
	return n;
}