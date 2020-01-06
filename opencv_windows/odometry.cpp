#include "odometry.h"


static bool setAimPt = false;
static Point2f AimPoint;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		AimPoint = Point2f((float)x, (float)(y));
		setAimPt = true;
	}
}


odometry::odometry()
{
	needToInitKeypoints = true;
	step_butch = 3;
	magnify_vektor_draw = 5;

	//hist =  histogram(10, "Steps");

	FileStorage ks("out_camera_data.xml", FileStorage::READ); // Read the settings
	if (!ks.isOpened())
	{
		cout << " Camera Matrix frei" << endl;
		float data[10] = { 700, 0, 320, 0, 700, 240, 0, 0, 1 };

		cameraMatrix = Mat(3, 3, CV_32FC1, data); // rows, cols

		float data1[5] = { -4.1802327018241026e-001, 5.0715243805833121e-001, 0., 0.,
			-5.7843596847939704e-001 };

		distCoeffs = Mat(5, 1, CV_32FC1, data1); // rows, cols
	}
	else
	{
		ks["camera_matrix"] >> cameraMatrix;
		ks["distortion_coefficients"] >> distCoeffs; //TODO
	}

	namedWindow(main_window_name, WINDOW_NORMAL | WINDOW_KEEPRATIO );

	setMouseCallback(main_window_name, onMouse, 0);

}

odometry::~odometry()
{
}

void odometry::find_keypoints()
{
	vector<Point2f> key_points;

	// Terminate Kriterium
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);

	Size subPixWinSize = Size(6, 6);

	// automatic initialization
	kp.clear();

	//finde features
	goodFeaturesToTrack(gray, key_points, kp.MAX_COUNT, 0.03, 10, Mat(), 9, 5);

	//refine position
	cornerSubPix(gray, key_points, subPixWinSize, Size(-1, -1), termcrit);

	kp.prev_points = key_points;

	// fülle keypunkte
	for (Point2f p : key_points)
	{											  
		kp.point.push_back(keypoint(p));
	}
	
}

void odometry::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	fokus.x = (float)(frame->cols / 2);
	fokus.y = (float)(frame->rows / 2); //TODO nur einmal machen

	swap();	// vorbereite vorherige Punkte für nächste Berechnung

	//kopieren in Abbildung
	frame->copyTo(image);

	// TODO Umrechnen 
	//	undistort(*frame, image, cameraMatrix, distCoeffs); //TODO nur auf Punkte anwenden

	cvtColor(image, gray, COLOR_BGR2GRAY);	//OPTI kann man vorher machen
}

//TODO erstellen Funktion übergabevariablen, z.B. Abweichung von der Linie
void odometry::check_for_followed_points(vector<uchar>* status, vector<float>* err)
{
	// löschen schlechte punkte 

	int number_followed_points = 0;

	// die punkte die kann man folgen werden gezählt 
	// und kopiert in point
	int index = 0;
	int n = 0;
	keypoint p;

	for (int i : *status)
	{
		if (i == 1)	//TODO Bedingung, Punkt ist in Ordnung
		{
			//letzte punkte in point
			p.set_position(kp.current_points[n]);
			//uebertrage alte Verschiebung
			p.shift_flow();
			//berechne aktuelle Verschiebung
			p.set_flow(kp.current_points[n] - kp.prev_points[n]);

			kp.point[index] = p;
			kp.current_points[index] = kp.current_points[n];
			kp.prev_points[index] = kp.prev_points[n];

			index++;
			number_followed_points++;
		}			
		else
		{
			;
		}

		n++;
	}

	kp.point.resize(index);
	kp.current_points.resize(index);
	kp.prev_points.resize(index);
	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	if (number_followed_points < MIN_FOLLOWED_POINTS) needToInitKeypoints = true; 
}

void odometry::find_follow_points()
{
	// Terminate Kriterium für die LukasKande
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);

	Size  winSize = Size(11, 11);  // SubWindow für LukasKande

	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK
	
	if (!kp.prev_points.empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray); // Absicherung

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ kp.prev_points, /*next*/ kp.current_points,
			status, err, winSize, 5, termcrit, 0, 0.001);

		check_for_followed_points(&status, &err);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		// Affine = estimateRigidTransform(kp.prev_points, kp.current_points, true);
		//Was ich eigentlich tue

		//cout << "Affine: " << Affine.at<double>(0, 2) << " - " << Affine.at<double>(1, 2) << endl << endl;
	}
}

//void odometry::transform_Affine()
//{
//	if (!Affine.empty() && !kp.prev_points.empty())
//	{
//		// Affine_x_last = Affine_x;
//
//		//Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
//											// umrechnen feautures
//		//	transform(kp.prev_points, kp.calculated_points[0], Affine);
//
//	}
//}

void odometry::draw_prev_points()
{
	for (Point2f p : kp.prev_points)
			circle(image, (Point)p, 3, Scalar(255, 0, 255));

}

void odometry::draw_current_points()
{
	int i = 0;

	for (Point2f p : kp.current_points)
	{
		circle(image, (Point)p, 3, Scalar(255, 250, 0));
	}

	int m = 0;

	for (int n : kp.numbers_of_downpoints)
	{
		stringstream text;

		text << format("%.0f",kp.step_length[m]);

		if (kp.current_points[n].x <  (gray.cols - 40))
		{
			putText(image, text.str(), kp.current_points[n] + Point2f(3,-3), 
				FONT_HERSHEY_PLAIN, 1.0f, Scalar(200, 200, 200));
		}

		m++;
	}

	// Markierung den Punkten mit gleichem Schritt
	for (size_t i = 0; i < kp.same_step_pt.size(); i++)
	{
		circle(image, (Point)kp.current_points[kp.same_step_pt[i]], 10, Scalar(0, 0, 160));
	}

}

//void odometry::draw_calculated_points()
//{
//	for (size_t i = 0; i < kp.calculated_points[0].size(); i++) // TODO was?
//	{
//		circle(image, (Point)kp.calculated_points[0][i], 4, Scalar(0, 0, 250));
//	}
//}

void odometry::draw_main_points()
{
	for (size_t i = 0; i < kp.background_points.size(); i++) 
	{
		circle(image, (Point)kp.current_points[kp.background_points[i]], 7, Scalar(0, 200, 0));
	}
}

void odometry::draw_summ_vector()
{
	Point2f p1, p2, p3;
	p2 = fokus;

	while (!kp.summ_queue_empty())
	{
		p3 = p2 + 10*magnify_vektor_draw * kp.get_next_summ_vector();

		line(image, (Point)p2, (Point)p3, Scalar(220, 220, 0), 3);
		circle(image, (Point)p2, 3, Scalar(0, 255, 0), 1);
		p2 = p3;
	};
}

int odometry::draw_image()
{
	int time = 10; //ms

	// draw Zielpunkt wenn gibt es 
	//draw_aim_point();

	//Draw die Punkte die entsprechen hintegrundvector
	//draw_main_points();

	// draw_prev_points();

	draw_current_points();

	//draw_nearest_point();

	draw_step_vectors();

	draw_flow();

	draw_summ_vector();

	time = 0;

	

	return time;
}

void odometry::draw_step_vectors() // batch
{
	Point2f p0, p1;
	p1 = Point2f(0, 0);

	for (int i = 0; i < kp.current_points.size(); i++)
	{

		p0 = kp.current_points[i];

		while (!kp.step_vector_empty(i))
		{
			p1 = p0 + magnify_vektor_draw * kp.get_next_step_vector(i); //HACK entnahme aus queue vector

			line(image, (Point)p0, (Point)(p1), Scalar(0, 0, 250));

			p0 = p1;
		};

	}

}

void odometry::draw_nearest_point()

{
	float* move = new float[kp.MAX_COUNT];

	float dist = 0.0f;
	float max_move = 0.0f;
	int neahrest_point = 0;
	bool danger = false;

}

void odometry::show_image()
{
	stringstream text;

	text.width(5);
	text.precision(3);

	text << "calc " << kp.prev_points.size() << " | " << kp.current_points.size() << " | " << "__" << frame_time;

	//putText(image, text.str(), Point(100, 100), FONT_HERSHEY_PLAIN, 2.0f, Scalar(0, 0, 0), 2);

	setWindowTitle(main_window_name, text.str());

	imshow(main_window_name, image);

}

void odometry::kompensate_jitter(vector<int>* points_number)
{
	kp.kompensate_jitter(points_number);
	main_of_frame = kp.main_jitter.back();
	//cout << "main: " << main.x << " - " << main.y << endl << endl;
}


void odometry::swap()
{
	kp.swap();

	cv::swap(prevGray, gray);
}

bool odometry::key(int wait)
{
	char c = (char)waitKey(wait);

	if (c == 27) return true;

	switch (c)
	{
	case 'r':
		needToInitKeypoints = true;
		break;

	case 'c':
		kp.clear();
		break;

	//case 'k':
	//	cam_calibrate();
	}

	return false;
}
/*
void odometry::look_to_aim()
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

		//s.correction(richtung);
		//TODO ueber Verbindung realisieren

//		cout << kp.current_points[number_aim_point] << m << richtung << "|" << s.position << endl;
	}
	else
	{
		cout << "find " << kp.current_points[number_aim_point] << m << richtung << endl;
		number_aim_point = -1;
		needToInitKeypoints = true;
	}



}
*/
int odometry::find_nearest_point(Point2f pt)
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
		if (d < dist )
		{
			dist = d;
			n = i;
		}

	}
	return n;
}

int odometry::collect_step_vectors()
{

	int number_followed_points = kp.save_step_vectors();
	return 0;
}

void odometry::find_backround_points(vector<int>* backround_points_numbers)
{
	int index = 0;
	for (keypoint p : kp.point)
	{
		if (p.get_position().y < fokus.y)  backround_points_numbers->push_back(index);
		index++;
	}

}

void odometry::draw_flow()
{
	Point2f p0, p1;
	p1 = Point2f(0, 0);

	for (keypoint p : kp.point)
	{
		p0 = p.get_position();

		int i = 0;

		while (i < 1/*ANZAHL_VORPOSITIONS*/)
		{
			p1 = p0 + magnify_vektor_draw * p.get_flow(i++); //OPTI verbessern

			line(image, (Point)p0, (Point)(p1), Scalar(0, 0, 250), 2);

			p0 = p1;
		};

	}

}



bool odometry::proceed_keypointsset(Mat* frame, std::vector <keypoints_flow>* key_points)
{
	vector<int> backround_points_numbers;

	take_picture(frame);

	imshow(main_window_name, *frame);

	kp.clear();

	// "Keypointsflow wird zerlegt auf Aktuelle Keypoints und vorherige und dann wird verarbeitet
	for (keypoints_flow p : *key_points)
	{
		kp.current_points.push_back(p.p);
		kp.prev_points.push_back(p.p - p.flow[0]);
	}

	kp.calc_distances();

	collect_step_vectors();

//	transform_Affine();

	find_backround_points(&backround_points_numbers);

	kompensate_jitter(&backround_points_numbers);

	int wait_time = draw_image();

	show_image();

//	look_to_aim();

	if (key(wait_time)) return true;

	return false;
}

bool odometry::proceed_video(Mat* frame)
{
	vector<int> backround_points_numbers;

	take_picture(frame);

	imshow(main_window_name, *frame);

	//kp.clear();

	if (needToInitKeypoints)
	{
		find_keypoints();
		needToInitKeypoints = false;
		return false;
	}

	find_follow_points();

	find_backround_points(&backround_points_numbers);

	kompensate_jitter(&backround_points_numbers);

	// kp.calc_distances();

	// collect_step_vectors();

	//	transform_Affine();

	int wait_time = draw_image();

	show_image();

	//	look_to_aim();

	if (key(wait_time)) return true;

	return false;
}

void odometry::new_data_proceed(UDP_Base* udp_base)
{


	//udp_base->udp_data_received();

	//send antwort an server 
}


