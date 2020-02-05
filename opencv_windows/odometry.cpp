#include "odometry.h"


static bool setAimPt = false;
static Point2f AimPoint;

#define MIN_FOLLOWED_POINTS 100

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		AimPoint = Point2f((float)x, (float)(y));
		setAimPt = true;
	}
}


odometry::odometry(Mat* frame)
{
	needToInitKeypoints = true;
	step_butch = 3;
	magnify_vektor_draw = 2;

	set_fokus(frame);

	FileStorage ks("../out_camera_data.xml", FileStorage::READ); // Read the settings
	if (!ks.isOpened())
	{
		cout << " ..//out_camera_data.xml nicht gefunden" << endl;

		exit(5);

		//float data[10] = { 700, 0, 320, 0, 700, 240, 0, 0, 1 };

		//cameraMatrix = Mat(3, 3, CV_32FC1, data); // rows, cols

		//float data1[5] = { -4.1802327018241026e-001, 5.0715243805833121e-001, 0., 0.,
		//	-5.7843596847939704e-001 };

		//distCoeffs = Mat(5, 1, CV_32FC1, data1); // rows, cols
	}
	else
	{
		
		ks["camera_matrix"] >> cameraMatrix;
		ks["distortion_coefficients"] >> distCoeffs; //TODO

		if (cameraMatrix.at<double>(0, 2) != fokus.x || cameraMatrix.at<double>(1, 2) != fokus.y)
		{
			cerr << "Calibrieren Aufloesung passt nicht zu ..//out_camera_data.xml" << endl;
			cout << cameraMatrix.at<double>(0, 2) << "x" << cameraMatrix.at<double>(1, 2) << endl;

			/*exit(6);*/  //HACK
		}

	}

	namedWindow(main_window_name, WINDOW_NORMAL | WINDOW_KEEPRATIO );

	setMouseCallback(main_window_name, onMouse, 0);

}

odometry::~odometry()
{
}

void odometry::set_fokus(Mat* frame)
{
	fokus.x = (float)(frame->cols / 2);
	fokus.y = (float)(frame->rows / 2); 

	kp.set_fokus(fokus);
}

void odometry::find_keypoints()
{
	vector<Point2f> key_points;

	// Terminate Kriterium
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);

	Size subPixWinSize = Size(18, 18);

	//finde features
	goodFeaturesToTrack(gray, key_points, kp.MAX_COUNT, 0.03, 10, Mat(), 15, 5);

	//refine position
	cornerSubPix(gray, key_points, subPixWinSize, Size(-1, -1), termcrit);

	// automatic initialization
	kp.clear();

	// fülle keypunkte
	for (Point2f p : key_points)
	{											  
		kp.point.push_back(keypoint(p));
	}

	
}

void odometry::find_keypoints_FAST()
{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	vector<Point2f> key_points;

	int fast_threshold = 20;
	bool nonmaxSuppression = true;

	AGAST(gray, keypoints_1, fast_threshold, nonmaxSuppression, AgastFeatureDetector::OAST_9_16);
	//FAST(gray, keypoints_1, fast_threshold, nonmaxSuppression);

	KeyPoint::convert(keypoints_1, key_points, vector<int>());

	// automatic initialization
	kp.clear();

	// fülle keypunkte
	for (Point2f p : key_points)
	{
		kp.point.push_back(keypoint(p));
	}

}

void odometry::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	swap();	// vorbereite vorherige Punkte für nächste Berechnung

	//kopieren in Abbildung
	frame->copyTo(image);

	//undistort(*frame, image, cameraMatrix, distCoeffs); //TODO nur auf Punkte anwenden  //HACK

	cvtColor(image, gray, COLOR_BGR2GRAY);	//OPTI kann man vorher machen

	//gray.copyTo(image);	//HACK
}

void odometry::check_for_followed_points(vector<Point2f>* prev_points, vector<Point2f>* current_points, vector<uchar>* status, vector<float>* err)
{															      
	int number_followed_points = 0;

	// die punkte die kann man folgen werden gezählt 
	// und kopiert in point
	int index = 0;
	int n = 0;
	keypoint p;

	for (int st : *status)
	{
		if (st == 1)	// Punkt ist in Ordnung
		{
			p = kp.point[n];
			//letzte punkte in point
			p.set_position((*current_points)[n]);
			//Verschiebe alte Werte
			p.shift_flow();
			//berechne aktuelle Verschiebung
			p.set_flow((*current_points)[n] - (*prev_points)[n]);

			kp.point[index] = p;
			//kp.prev_points[index] = kp.current_points[n];
			//kp.prev_points[index] = kp.prev_points[n];

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

	//TODO Punkte auf Linie ueberpruefen

	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	if (number_followed_points < MIN_FOLLOWED_POINTS) needToInitKeypoints = true; 
}

void odometry::find_followed_points()
{
	// Terminate Kriterium für die LukasKande
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

	Size  winSize = Size(21, 21);  // SubWindow für LukasKande

	vector<uchar> status; // status vom calcOpticalFlowPyrLK
	vector<float> err; // error vom calcOpticalFlowPyrLK
	vector<Point2f> prev_points; // vorherige punkte
	vector<Point2f> current_points; // aktuelle punkte

	prev_points.clear();

	//Befüllen punkte zum vergleich	//OPTI

	for (keypoint p : kp.point)
	{
		prev_points.push_back(p.get_position());
	}
	
	if (!prev_points.empty())
	{
		//if (prevGray.empty()) gray.copyTo(prevGray); // Absicherung

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ prev_points, /*next*/ current_points,
			status, err, winSize, 5, termcrit, 0, 0.001);

		check_for_followed_points(&prev_points, &current_points, &status, &err);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		// Affine = estimateRigidTransform(kp.prev_points, kp.current_points, true);
		//Was ich eigentlich tue

		//cout << "Affine: " << Affine.at<double>(0, 2) << " - " << Affine.at<double>(1, 2) << endl << endl;
	}
	else
	{
		needToInitKeypoints = true;
		cerr << "Keine punkte zum Verfolgen" << endl;
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


void odometry::draw_keypoints()
{
	kp.draw(&image);
}

void odometry::draw_background_points()
{
	kp.draw_background_points(&image);
}

void odometry::draw_ground_points()
{
	kp.draw_ground_points(&image);
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
		circle(image, (Point)kp.point[kp.background_points[i]].get_position(), 7, Scalar(0, 200, 0));
	}
}

void odometry::draw_summ_vector()
{
	Point2f p1, p2, p3;
	p2 = fokus;

	while (!kp.summ_queue_empty())
	{
		p3 = p2 + magnify_vektor_draw * kp.get_next_summ_vector();

		line(image, (Point)p2, (Point)p3, Scalar(220, 220, 0), 3);
		circle(image, (Point)p2, 3, Scalar(0, 255, 0), 3);
		p2 = p3;
	};
}

int odometry::draw_image()
{
	int time = 3000; //ms

	// draw Zielpunkt wenn gibt es 
	//draw_aim_point();

	//Draw die Punkte die entsprechen hintegrundvector
	//draw_main_points();

	// draw_prev_points();

	draw_keypoints();

	draw_background_points();

	draw_ground_points();

	//draw_nearest_point();

	draw_flow();

	draw_summ_vector();

	//time = 0;

	show_image();

	return time;
}

void odometry::show_image()
{
	stringstream text;

	text.width(5);
	text.precision(3);

	text << "calc " << kp.point.size() << " | " << frame_number;

	//putText(image, text.str(), Point(100, 100), FONT_HERSHEY_PLAIN, 2.0f, Scalar(0, 0, 0), 2);

	setWindowTitle(main_window_name, text.str());

	imshow(main_window_name, image);

}

void odometry::kompensate_jitter()
{
	kp.kompensate_jitter();
	main_of_frame = kp.main_jitter.back();
	//cout << "main: " << main.x << " - " << main.y << endl << endl;
}

void odometry::kompensate_roll()
{
	kp.kompensate_roll();
	//cout << "main: " << main.x << " - " << main.y << endl << endl;
}

void odometry::swap()
{
	cv::swap(prevGray, gray);
}

bool odometry::key(int wait)
{
	static int  wait_time = 0;

	char c = (char)waitKey(wait_time);

	if (c == 27) return true;

	switch (c)
	{

	case 's':
		wait_time = 0;
		break;

	case 'g':
		wait_time = 1000;
		break;

	case 'r':
		needToInitKeypoints = true;
		break;

	default:
		break;
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
//int odometry::find_nearest_point(Point2f pt)
//{
//	float d, dist = 10000000.0;
//	Point2f v;
//	int n = 0; //TODO wenn 0 bearbeiten
//
//	n = 0;
//	for (int i = 0; i < kp.current_points.size(); i++)
//	{
//		// draw berechnete features
//		v = kp.current_points[i] - pt;
//		d = v.x*v.x + v.y*v.y;
//		if (d < dist )
//		{
//			dist = d;
//			n = i;
//		}
//
//	}
//	return n;
//}

void odometry::find_background_points()
{
	int index = 0;
	Point2f d;
	int l = 300;

	kp.background_points.clear();

	for (keypoint p : kp.point)
	{
		d = p.get_position();

		if (d.y < l  && d.x < fokus.x + l && d.x > fokus.x - l)
		{
			kp.background_points.push_back(index);
		}
			
		index++;
	}

}

void odometry::find_ground_points()
{
	int index = 0;
	Point2f d;
	int l = 300;

	kp.ground_points.clear();

	for (keypoint p : kp.point)
	{
		d = p.get_position();

		if (d.y > 2*fokus.y - l && d.x < fokus.x + l && d.x > fokus.x - l)
		{
			kp.ground_points.push_back(index);
		}

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

		while (i < ANZAHL_VORPOSITIONS)  //HACK
		{
			p1 = p0 + magnify_vektor_draw * p.get_flow(i++); //OPTI verbessern

			line(image, (Point)p0, (Point)(p1), Scalar(0, 0, 250), 2);

			p0 = p1;
		};

		line(image, (Point)p.get_position(), (Point)(p.get_position() + magnify_vektor_draw * p.get_full_flow()), Scalar(250, 0, 0), 2);

	}

}

//bool odometry::proceed_keypointsset(Mat* frame, std::vector <keypoints_flow>* key_points)
//{
//	vector<int> backround_points_numbers;
//
//	take_picture(frame);
//
//	imshow(main_window_name, *frame);
//
//	kp.clear();
//
//	// "Keypointsflow" wird zerlegt auf Aktuelle Keypoints und vorherige und dann wird verarbeitet
//	for (keypoints_flow p : *key_points)
//	{
//		kp.current_points.push_back(p.p);
//		kp.prev_points.push_back(p.p - p.flow[0]);
//	}
//
//	kp.calc_distances_1(fokus);
//
//	//collect_step_vectors();
//
////	transform_Affine();
//
//	find_backround_points();
//
//	kompensate_jitter();
//
//	int wait_time = draw_image();
//
//	show_image();
//
////	look_to_aim();
//
//	if (key(wait_time)) return true;
//
//	return false;
//}
//
bool odometry::proceed_video(Mat* frame)
{

	int wait_t = 0;

	take_picture(frame);

	frame_number++;

	imshow(main_window_name, *frame);

	if (needToInitKeypoints)
	{
		//find_keypoints_FAST();
		find_keypoints();
		needToInitKeypoints = false;
		return false;
	}

	find_followed_points();

	find_background_points();

	find_ground_points();

	kompensate_jitter();
	//kompensate_roll();

	float step = kp.calc_step();

	kp.calc_distances(step);

	//	transform_Affine();

	draw_image();

	if (key(wait_t)) return true;

	return false;
}

void odometry::new_data_proceed(UDP_Base* udp_base)
{


	//udp_base->udp_data_received();

	//send antwort an server 
}


