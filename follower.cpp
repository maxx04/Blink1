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
		kp.clear();

		//finde features
		goodFeaturesToTrack(gray, kp.prev_points, kp.MAX_COUNT, 0.05, 12, Mat(), 5, 5, 0, 0.04);

		//refine position
		cornerSubPix(gray, kp.prev_points, subPixWinSize, Size(-1, -1), termcrit);

		needToInit = false;
	}
}

void follower::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	fokus.x = (float)(image.cols / 2);
	fokus.y = (float)(image.rows / 2); //TODO nur einmal machen
	swap();
	frame->copyTo(image);

	kp.frame_timestamp.push((double)getTickCount());

	cvtColor(image, gray, COLOR_BGR2GRAY);
}

void follower::calcOptFlow()
{


	if (!kp.prev_points.empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray);

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ kp.prev_points, /*next*/ kp.current_points,
			kp.status, kp.err, winSize, 3, termcrit, 0, 0.001);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		kp.load_queue();

		Affine = estimateRigidTransform(kp.prev_points, kp.current_points, true);

		//needToInit = true;
	}
}

void follower::transform_Affine()
{
	if (!Affine.empty() && !kp.prev_points.empty())
	{
		// Affine_x_last = Affine_x;

		//Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
											// umrechnen feautures
		transform(kp.prev_points, kp.calc[0], Affine);

	}
}

int follower::draw()
{
	int time = 10; //ms


	// previous punkte 
	for (size_t i = 0; i < kp.prev_points.size(); i++) // TODO
	{
		// draw berechnete features
		if (kp.status[i] == 1)
		{
			circle(image, (Point)kp.prev_points[i], 4, Scalar(255, 0, 255));
		}
		else
			circle(image, (Point)kp.prev_points[i], 4, Scalar(255, 0, 0));


	}




	if (number_aim_point >= 0)

		circle(image, (Point)kp.current_points[number_aim_point], 16, Scalar(0, 0, 255), 3);

	circle(image, (Point)AimPoint, 16, Scalar(0, 255, 0), 3);

	double Affine_x = 0.0;

	if (!Affine.empty() && !kp.prev_points.empty())
	{
		Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
	}

	// aktuelle punkte



	Point2f p2, p3;

	if (kp.p_sum.size() == queue_size)
	{

		frame_time = kp.get_queue_time();


		// draw summ vector
		p2 = fokus;
		while (!kp.p_sum.empty())
		{
			p3 = p2 + 1.0*kp.p_sum.front();
			kp.p_sum.pop();
			line(image, (Point)p2, (Point)p3, Scalar(0, 0, 200), 3);
			circle(image, (Point)p2, 3, Scalar(0, 255, 0), 1);
			p2 = p3;
		};

		int i = 0;
		int n_status = 0;
		float* move = new float[kp.MAX_COUNT];

		float dist = 0.0f;
		float max_move = 0.0f;
		int neahrest_point = 0;
		bool danger = false;

		for (i = 0; i <  kp.prev_points.size(); i++)
		{
			if (kp.status[i] == 1)
			{
				n_status++;
				Point2f p0, p1;

				p0 = kp.prev_points[i];

				while (!kp.points_queue[i].empty())
				{
					p1 = p0 + kp.points_queue[i].front();
					kp.points_queue[i].pop();
					line(image, (Point)p0, (Point)p1, Scalar(255, 255, 100));
					circle(image, (Point)p0, 1, Scalar(0, 255, 0), 1);
					dist += kp.distance(p0, p1);

					p0 = p1;
				};

				move[i] = dist; // Distance für letzten queue_size punkten
				

				if (dist > max_move)
				{
					max_move = dist;
					neahrest_point = i;
				}

				dist = 0.0;
			}

		}

		if (max_move > 10.0f)
		{
			danger = true;
			// zeichne naeheresten punkt
			circle(image, (Point)kp.prev_points[neahrest_point], 10, Scalar(0, 0, 200), 3);
		}


 		time = 10;

		if (n_status < 100)
		{
			needToInit = true;
		}
	}

	return time;
}

void follower::show()
{
	stringstream text;

	text.width(5);
	text.precision(3);

	//text << "calc " << timeSec * 1000 << " ms " << "  " << points[1].size() << "  " <<  Affine_x * 10;

	int st = 0;

	for (size_t i = 0; i < kp.status.size(); i++)
	{
		st += kp.status[i];
	}

	text << "calc " << kp.prev_points.size() << " | " << kp.current_points.size() << " | " << st << "__" << frame_time;


	//putText(image, text.str(), Point(100, 100), FONT_HERSHEY_PLAIN, 2.0f, Scalar(0, 0, 0), 2);

//#ifndef _ARM

	setWindowTitle("LK Demo", text.str());

	imshow("LK Demo", image);

	//#endif
}

void follower::cam_calibrate()
{
	Size_<int> boardSize(7,7);
	vector<Point2f> pointBuf;
	vector<vector<Point3f> > objectPoints(1);


	//Find intrinsic and extrinsic camera parameters
	double rms;

	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
	bool found = findChessboardCorners(image, boardSize, pointBuf, chessBoardFlags);
	if (!found) return;

	//calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

	//objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
	//	s.flag);

	string a = "r";
	char* arguments[2];
	arguments[0] = &a[0];

	calibrate(0, arguments);

	return;
}

void follower::swap()
{
	kp.swap();

	cv::swap(prevGray, gray);
}

bool follower::key(int wait)
{
	char c = (char)waitKey(wait);

	if (c == 27) return true;

	switch (c)
	{
	case 'r':
		needToInit = true;
		break;

	case 'c':
		kp.clear();
		break;

	case 'k':
		cam_calibrate();
	}

	return false;
}

void follower::look_to_aim()
{
	Point2f richtung = Point2f(0.0, 0.0);
	Point2f m;

	if (setAimPt)
	{
		// 1) finde positiondifferenz
		number_aim_point = find_nearest_point(AimPoint);
		setAimPt = false;
	}

	if (number_aim_point < 0) return;

	// 1) finde positiondifferenz
	m = kp.current_points[number_aim_point] - fokus;

	// 2) finde richtung
	richtung.x = -m.x / pixel_pro_step;
	richtung.y = m.y / pixel_pro_step;
	// 3a) finde wo ist jetzt den Punkt (z.B. über Matrix)
	// kontrolle über vergleich p[0] - P[1]
	// 4) wenn differenz immer noch groß, gehe zu schritt 1. 
	// 3) bewege einen schritt in Richtung
	if (abs(m.x) > 20.0 || abs(m.y) > 20.0)
	{

		s.correction(richtung);

		cout << kp.current_points[number_aim_point] << m << richtung << "|" << s.position << endl;
	}
	else
	{
		cout << "find " << kp.current_points[number_aim_point] << m << richtung << endl;
		number_aim_point = -1;
		needToInit = true;
	}



}

int follower::find_nearest_point(Point2f pt)
{
	float d, dist = 10000000.0;
	Point2f v;
	int n = 0; //TODO wenn 0 bearbeiten

	for (int i = 0; i < kp.prev_points.size(); i++)
	{
		// draw berechnete features
		v = kp.prev_points[i] - pt;
		d = v.x*v.x + v.y*v.y;
		if (d < dist && kp.status[i] == 1)
		{
			dist = d;
			n = i;
		}

	}
	return n;
}
