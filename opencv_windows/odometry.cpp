#include "odometry.h"

static bool setAimPt = false;
static Point2f AimPoint;


static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		AimPoint = Point2f((float)x, (float)(y));
		cout << AimPoint << endl;
		setAimPt = true;
	}
}

odometry::odometry(Mat* frame)
{
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

		cout << cameraMatrix << endl << distCoeffs << endl;

		if (cameraMatrix.at<double>(0, 2) != fokus.x || cameraMatrix.at<double>(1, 2) != fokus.y)
		{
			cerr << "Calibrieren Aufloesung passt nicht zu ..//out_camera_data.xml" << endl;
			cout << cameraMatrix.at<double>(0, 2) << "x" << cameraMatrix.at<double>(1, 2) << endl;

			/*exit(6);*/  //HACK
		}

	}

	focal_length = cameraMatrix.at<double>(0, 0);
	VFOV2 = atan(fokus.y / focal_length);
	HFOV2 = atan(fokus.y / focal_length);
	cam_v_distance = 118.0;
	cam_pitch = 3.8357 * M_PI / 180.0;
	yaw_angle = 0.0;
	pitch_angle = 0.0f;
	current_position = Point2f(0,0);
	needToInitKeypoints = true;
	magnify_vektor_draw = 3;

	namedWindow(main_window_name, WINDOW_NORMAL | WINDOW_KEEPRATIO);

	setMouseCallback(main_window_name, onMouse, 0);

}

odometry::~odometry()
{
}

void odometry::set_fokus(Mat* frame)
{
	fokus.x = (float)(frame->cols / 2);
	fokus.y = (float)(frame->rows / 2);

}

void odometry::find_keypoints()
{
	vector<Point2f> key_points;// , key_points_ud;

	// Terminate Kriterium
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);

	Size subPixWinSize = Size(18, 18);

	//finde features
	goodFeaturesToTrack(prevGray, key_points, kp.MAX_COUNT, 0.03, 10, Mat(), 15, 5);

	//refine position
	cornerSubPix(prevGray, key_points, subPixWinSize, Size(-1, -1), termcrit);

	//	undistortPoints(key_points, key_points_ud, cameraMatrix, distCoeffs); //HACK

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
	vector<Point2f> key_points;// , key_points_ud;

	int fast_threshold = 20;
	bool nonmaxSuppression = true;

	//AGAST(prevGray, keypoints_1, fast_threshold, nonmaxSuppression, AgastFeatureDetector::OAST_9_16);
	FAST(prevGray, keypoints_1, fast_threshold, nonmaxSuppression);

	KeyPoint::convert(keypoints_1, key_points, vector<int>());

	//HACK nur notwendige punkte in Zukunft benutzen
	//undistortPoints(key_points, key_points_ud, cameraMatrix, distCoeffs); 

	// automatic initialization
	kp.clear();

	// fülle keypunkte
	for (Point2f p : key_points)
	{
		kp.point.push_back(keypoint(p));
	}

}

void odometry::find_keypoints_ORB()
{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	vector<Point2f> key_points;// , key_points_ud;

	//int fast_threshold = 20;
	//bool nonmaxSuppression = true;

	Mat descriptors1;

	Ptr<Feature2D> orb = ORB::create(kp.MAX_COUNT);
	orb->detectAndCompute(prevGray, Mat(), keypoints_1, descriptors1);

	KeyPoint::convert(keypoints_1, key_points, vector<int>());

	//HACK nur notwendige punkte in Zukunft benutzen
	//undistortPoints(key_points, key_points_ud, cameraMatrix, distCoeffs); 

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

	cv::swap(gray, prevGray); // erhalte vorheriges Bild 

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
			p.set_position((*current_points)[n]); //OPTI nur einmal uebergeben current
			//Verschiebe alte Werte
			p.shift_flow();
			//berechne aktuelle Verschiebung
			p.set_flow((*current_points)[n] - (*prev_points)[n]);

			kp.point[index] = p;

			index++;
			number_followed_points++;
		}

		n++;
	}

	kp.point.resize(index);

	kp.point[0].set_step();	// setzen befuellungsgrad vom flow

	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	if (number_followed_points < min_followed_points) needToInitKeypoints = true;
}

void odometry::find_followed_points()
{
	// Terminate Kriterium für die LukasKande
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

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
		// Was ich eigentlich tue

		//cout << "Affine: " << Affine.at<double>(0, 2) << " - " << Affine.at<double>(1, 2) << endl << endl;
	}
	else
	{
		needToInitKeypoints = true;
		cerr << "Keine punkte zum Verfolgen" << endl;
	}

}

void odometry::find_obstacles()
{

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

//void odometry::draw_main_points()
//{
//	for (size_t i = 0; i < kp.background_points.size(); i++) 
//	{
//		circle(image, (Point)kp.point[kp.background_points[i]].get_position(), 7, Scalar(0, 200, 0));
//	}
//}

void odometry::draw_summ_vector()
{
	Point2f p1, p2, p3;


	p2 = fokus;
	p3 = p2 + magnify_vektor_draw * kp.main_jitter[kp.main_jitter.size() - 1];

	line(image, (Point)p2, (Point)p3, Scalar(220, 220, 0), 1);
	circle(image, (Point)p2, 2, Scalar(0, 255, 0), -3);

	//for (Point2f p: kp.main_jitter)
	//{
	//	p3 = p2 + magnify_vektor_draw * p;

	//	line(image, (Point)p2, (Point)p3, Scalar(220, 220, 0), 1);
	//	circle(image, (Point)p2, 2, Scalar(0, 255, 0), -3);
	//	p2 = p3;
	//}

}

void odometry::draw_map()
{
	map.draw_map(this);

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

	draw_map();

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

float odometry::calc_step()
{

	float beta, beta1;	// Winkel vom Mittelachse Kamera zu dem Punkt auf dem Boden [radian]
	float v1;  //Bild koordinate y für vorherige Position
	float distance;	// Abstand vom Kamera zu Punkt auf horizontale Ebene
	float current_pitch = atan(main_of_frame.y / focal_length);	  // [radian]

	kp.hist_step.clear();

	for (int i : kp.ground_points)
	{
		float v = kp.point[i].get_position().y; // Bildkoordinaten vom Schlüsselpunkt y

		beta = atan((v / fokus.y - 1.0f) * tan(VFOV2)); // OPTI tan_beta lassen / Formel (2) tan(b) = (2*v/V-1)*tan(VFOV/2)

		distance = cam_v_distance / tan(cam_pitch + beta - current_pitch); // OPTI cos(alfa); tan(alfa) vorberechnen	// TODO assert	alfa + beta = pi/2

		float v1 = v - kp.point[i].get_flow(0).y;

		beta1 = atan((v1 / fokus.y - 1.0f) * tan(VFOV2)); // OPTI tan_beta lassen

		float step = cam_v_distance / tan(cam_pitch + beta1 - current_pitch) - distance;	 // Weg für den Schlüsselpunkt zwischen Frames	// TODO assert	alfa + beta = pi/2

		kp.hist_step.collect({ i, step }); //TODO grenzen verfeinern

	}

	kp.hist_step.sort();

	float middle_step = kp.hist_step.main_mean;

	Point2f p;

	p.y = middle_step * cos(yaw_angle / 180.0 * M_PI);
	p.x = -middle_step * sin(yaw_angle / 180.0 * M_PI);

	ego_moving.push_back(p);

	current_position += p;

	return middle_step;
}

void odometry::calc_relative_distances()
{
	//float L = step; // einen Schritt Vorwaerts

	assert(fokus.x != 0.0 && fokus.y != 0.0);

	Point2f v(0, 0);
	Point2f r, l, b;
	float length_l, length_r;


	for (int i = 0; i < kp.point.size(); i++)
	{

		r = kp.point[i].get_position() - fokus;

		length_r = kp.length(r);

		v = kp.point[i].get_full_flow();

		if (abs(r.x) > 2.0)
		{
			l = (v.x / r.x) * r;
		}
		else if (abs(r.y) > 0.0)
		{
			l = (v.y / r.y) * r;

			l.x = 0.0; //HACK
		}
		else
		{
			cout << "r ist 0.0" << endl;
		}

		length_l = kp.length(l);

		if (length_l != 0.0)
		{
			kp.point[i].set_vectors(l, v - l, (length_r / length_l + 1.0f));
		}

	}

	return;
}

void odometry::calc_kp_coordinates()
{
	float beta_x, beta_y;

	int i = 0;																						    

	for (keypoint p : kp.point)
	{
		beta_y = atan((p.get_position().y / fokus.y - 1.0f) * tan(VFOV2));

		p.rel_ground_pos.y = cam_v_distance / tan(cam_pitch + beta_y); // auf dem Boden

		//p.rel_ground_pos.y = p.d * (ego_moving[ego_moving.size()-1]).y;

		//beta_x = atan((p.get_position().x / fokus.x - 1.0f) * tan(HFOV2));

		p.rel_ground_pos.x = p.rel_ground_pos.y / focal_length * (p.get_position().x - fokus.x);

		p.rel_ground_pos.z = p.rel_ground_pos.y / focal_length * (p.get_position().y - fokus.y);

		kp.point[i] = p;

		i++;

	}
}

void odometry::find_background_points()
{
	int index = 0;
	Point2f d;
	int l = 300;

	kp.background_points.clear();

	for (keypoint p : kp.point)
	{
		d = p.get_position();

		if (d.y < fokus.y && d.x < fokus.x + l && d.x > fokus.x - l)
		{
			kp.background_points.push_back(index);
		}

		index++;
	}

	if (kp.background_points.size() < min_background_points)	 // HACK 
	{
		needToInitKeypoints = true;
	}

}

void odometry::find_ground_points()
{
	int index = 0;
	Point2f d;
	int l = 600;

	kp.ground_points.clear();

	for (keypoint p : kp.point)
	{
		d = p.get_position();

		if (d.y > fokus.y + 200.0 && d.x < fokus.x + l && d.x > fokus.x - l)
		{
			kp.ground_points.push_back(index);
		}

		index++;
	}

	if (kp.ground_points.size() < min_ground_points)	 // HACK korrigieren 
	{
		needToInitKeypoints = true;
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

		while (i < flow_steps)  //HACK
		{
			p1 = p0 + magnify_vektor_draw * p.get_flow(i++); //OPTI verbessern

			line(image, (Point)p0, (Point)(p1), Scalar(0, 0, 250), 1);

			p0 = p1;
		};

		//line(image, (Point)p.get_position(), (Point)(p.get_position() + magnify_vektor_draw * p.get_full_flow()), Scalar(250, 0, 0), 2);

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

	int trying = 0;

	do {

		if (needToInitKeypoints)
		{
			//cv::swap(prevGray, gray);  // letztes Bild wiederherstellen

			//find_keypoints_ORB();
			find_keypoints_FAST();
			//find_keypoints();
			needToInitKeypoints = false;

			//cv::swap(prevGray, gray);  // noch mal tauschen um Folgepunkte zu finden

			if (trying++ > 3)
			{
				cout << "versuche finden Keypunkte " << trying << endl;
				//return false;
				waitKey(0);
				//exit(8);
			}
		}

		find_followed_points();

		//OPTI undistort points

		find_background_points();

		kompensate_jitter();
		//kompensate_roll();

		kp.check_trajektory(); // Prüfen auf Linienform vom Flow, ausschliessen Keypoints

		find_background_points(); //OPTI falsche punkte vorhanden wegen trajektory

		find_ground_points();

	} while (needToInitKeypoints);

	trying = 0;

	find_yaw_pitch();  // fuer weitere berechnungen

	float step = calc_step();

	calc_relative_distances(); // Abstände zu Punkten berechnen

	calc_kp_coordinates();

	// finden Schritt aus vorherigen Position des Bodenpunkten 
	// denn die auf gleiche Position (Relativ zu Anfang) sollen bleiben
	// in 2d soll man dx, dy, delta-yaw finden.
	// bei neuer Suche wird schritt abgenullt

	find_obstacles();

	//transform_Affine();

	map.draw_map(this);

	draw_image();

	if (key(wait_t)) return true;

	return false;
}

void odometry::find_yaw_pitch()
{
		Point2f p;

		yaw_angle += atan(main_of_frame.x / focal_length) * 180.0 / M_PI;
		pitch_angle += atan(main_of_frame.y / focal_length) * 180.0 / M_PI;
		//cout << main_of_frame.x << "|" << yaw_angle << endl;


}

void odometry::new_data_proceed(UDP_Base* udp_base)
{


	//udp_base->udp_data_received();

	//send antwort an server 
}


