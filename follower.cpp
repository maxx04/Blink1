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
	termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
	subPixWinSize = Size(10, 10);
	winSize = Size(31, 31);

	needToInit = true;


	float data[10] = { 700, 0, 320, 0, 700, 240, 0, 0, 1 };

	cameraMatrix = Mat(3, 3, CV_32FC1, data); // rows, cols

	/***/

	Mat vec = Mat(3, 1, CV_32FC1, { 10.0, 12.0, 3.0 });

	Mat n = cameraMatrix * vec;

	cout << n << endl;

	/***/

	//#ifndef _ARM
	namedWindow("LK Demo", 1);

	setMouseCallback("LK Demo", onMouse, 0);
	//#endif
}

follower::~follower()
{
}

void follower::find_keypoints()
{

	// automatic initialization
	kp.clear();

	//finde features
	goodFeaturesToTrack(gray, kp.prev_points, kp.MAX_COUNT, 0.02, 12, Mat(), 5, 5, 0, 0.04);

	//refine position
	cornerSubPix(gray, kp.prev_points, subPixWinSize, Size(-1, -1), termcrit);

}

void follower::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	fokus.x = (float)(image.cols / 2);
	fokus.y = (float)(image.rows / 2); //TODO nur einmal machen
	swap();

	frame->copyTo(image);

	kp.frame_timestamp.push((double)getTickCount()); //TODO wenn video berechnen frames pro sec

	cvtColor(image, gray, COLOR_BGR2GRAY);
}

void follower::check_for_followed_points()
{
	int number_followed_points = 0;

	// die punkte die kann man volgen werden gezählt 
	for (int i : kp.status)
	{
		if (i == 1) number_followed_points++;
	}
	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	if (number_followed_points < 200) needToInit = true; // TODO 100 als parameter
}

void follower::calcOptFlow()
{
	if (!kp.prev_points.empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray);

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ kp.prev_points, /*next*/ kp.current_points,
			kp.status, kp.err, winSize, 3, termcrit, 0, 0.001);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		Affine = estimateRigidTransform(kp.prev_points, kp.current_points, true);
		//Was ich eigentlich tue

//		cout << "Affine: " << Affine.at<double>(0, 2) << " - " << Affine.at<double>(1, 2) << endl << endl;
	}
}

void follower::transform_Affine()
{
	if (!Affine.empty() && !kp.prev_points.empty())
	{
		// Affine_x_last = Affine_x;

		//Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
											// umrechnen feautures
		transform(kp.prev_points, kp.calculated_points[0], Affine);

	}
}

void follower::draw_aim_point()
{
	if (number_aim_point >= 0 && number_aim_point <= kp.current_points.size()) //TODO manchmal nummer out of size

		circle(image, (Point)kp.current_points[number_aim_point], 16, Scalar(0, 0, 255), 3);

	circle(image, (Point)AimPoint, 16, Scalar(0, 255, 0), 3);
}

void follower::draw_prev_points()
{
	for (size_t i = 0; i < kp.prev_points.size(); i++) // TODO
	{
		// draw berechnete features
		if (kp.status[i] == 1)
		{
			circle(image, (Point)kp.prev_points[i], 3, Scalar(255, 0, 255));
		}
		else
			circle(image, (Point)kp.prev_points[i], 3, Scalar(255, 0, 0));
	}
}

void follower::draw_current_points()
{
	for (Point2f p : kp.current_points)
	{
			circle(image, (Point)p, 3, Scalar(255, 250, 0));
	}
}

void follower::draw_calculated_points()
{
	for (size_t i = 0; i < kp.calculated_points[0].size(); i++) // TODO was?
	{
		circle(image, (Point)kp.calculated_points[0][i], 4, Scalar(0, 0, 250));
	}
}

void follower::draw_main_points()
{
	for (size_t i = 0; i < kp.background_points.size(); i++) // TODO
	{
		circle(image, (Point)kp.prev_points[kp.background_points[i]], 7, Scalar(0, 200, 0));
	}
}

void follower::draw_summ_vector()
{
	Point2f p1, p2, p3;
	p2 = fokus;
	while (!kp.summ_queue_empty())
	{
		p3 = p2 + 1.0 * kp.get_next_summ_vector();

		line(image, (Point)p2, (Point)p3, Scalar(0, 0, 200), 3);
		circle(image, (Point)p2, 3, Scalar(0, 255, 0), 1);
		p2 = p3;
	};
}

int follower::draw_image()
{
	int time = 10; //ms

	// draw Zielpunkt wenn gibt es 
	draw_aim_point();

	//Draw die Punkte die entsprechen hintegrundvector
	draw_main_points();


	if (kp.summ_vector.size() == step_butch) // wenn anzahl frames wird erreicht dann abbilden 
	{

		frame_time = kp.get_queue_time(); //Zeit für Frame holen

		draw_summ_vector();

		draw_step_vectors();

		draw_prev_points();

		draw_current_points();

		draw_nearest_point();

		//	draw_calculated_points();

		check_for_followed_points();

		time = 0; // und time 0 stop und warte auf tastatur

	}

	return time;
}

void follower::draw_step_vectors() // batch
{
	Point2f p0, p1;
	p1 = Point2f(0, 0);

	for (int i = 0; i < kp.prev_points.size(); i++)
	{
		//if (kp.status[i] == 1)
		//{
		p0 = kp.prev_points[i];

		while (!kp.step_vector_empty(i))
		{
			p1 = p0 + kp.get_next_step_vector(i); //HACK entnahme aus queue vector

			line(image, (Point)p0, (Point)(p1), Scalar(255, 255, 100));
			//circle(image, (Point)p0, 2, Scalar(0, 255, 0), 1);

			p0 = p1;
		};

		//}

	}

}

void follower::draw_nearest_point()

{

	float* move = new float[kp.MAX_COUNT];

	float dist = 0.0f;
	float max_move = 0.0f;
	int neahrest_point = 0;
	bool danger = false;

}

void follower::show_image()
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

	char a[32];
	char* arguments[2];
	arguments[0] = &a[0];

	cameraMatrix = calibrate(0, arguments);

	cout << cameraMatrix << endl;

	return;
}

void follower::calculate_move_vectors()
{
	kp.calculate_move_vectors();
	Point2f main = kp.summ_vector.back();
	//cout << "main: " << main.x << " - " << main.y << endl << endl;
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

	n = 0;
	for (int i = 0; i < kp.current_points.size(); i++)
	{
		// draw berechnete features
		v = kp.current_points[i] - pt;
		d = v.x*v.x + v.y*v.y;
		if (d < dist && kp.status[i] == 1)
		{
			dist = d;
			n = i;
		}

	}
	return n;
}

int follower::collect_step_vectors()
{

	int number_followed_points = kp.save_step_vectors();
	return 0;
}

// Bearbeitet Frame in schritten
bool follower::proceed_frame(Mat* frame)
{
	// TODO: Fügen Sie hier Ihren Implementierungscode ein..
	take_picture(frame);

	if (needToInit)
	{
		find_keypoints();
		needToInit = false;
		kp.swap();
		return false;
	}

	calcOptFlow();

//	check_for_followed_points(); //TODO zuerst finden die Punkte die gut sind (status) 
	//nur dann collect step vectors.

	collect_step_vectors(); //

	transform_Affine();

	calculate_move_vectors();

	int wait_time = draw_image();

	show_image();

	look_to_aim();

	if (key(wait_time)) return true;

	return false;
}
