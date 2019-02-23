#include "station.h"


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

station::station()
{
	termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03);
	subPixWinSize = Size(6, 6);
	winSize = Size(11, 11);

	needToInit = true;
	step_butch = 1;
	magnify_vektor_draw = 1;

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

	namedWindow("LK Demo", 1);

	setMouseCallback("LK Demo", onMouse, 0);

}

station::~station()
{
}

void station::find_keypoints()
{

	// automatic initialization
	kp.clear();

	//finde features
	goodFeaturesToTrack(gray, kp.prev_points, kp.MAX_COUNT, 0.03, 10, Mat(), 9, 5);

	//refine position
	cornerSubPix(gray, kp.prev_points, subPixWinSize, Size(-1, -1), termcrit);

}

void station::take_picture(Mat* frame)
{
	if (frame->empty())	return; //TODO Fehlerabarbeitung

	fokus.x = (float)(image.cols / 2);
	fokus.y = (float)(image.rows / 2); //TODO nur einmal machen

	swap();

	frame->copyTo(image);
	// Umrechnen 
//	undistort(*frame, image, cameraMatrix, distCoeffs); //TODO nur auf Punkte anwenden

	kp.frame_timestamp.push((double)getTickCount()); //TODO wenn video berechnen frames pro sec

	cvtColor(image, gray, COLOR_BGR2GRAY);
}

void station::check_for_followed_points()
{
	int number_followed_points = 0;
	int kp_nummer = 0;
	// die punkte die kann man volgen werden gezählt 
	for (int i : kp.status)
	{
		if (i == 1) number_followed_points++;
		else
		{
			//kp.current_points[kp_nummer].
		}
		kp_nummer++;
	}
	//wenn weniger als 100 Punkten dann neue Punkte erstellen.
	if (number_followed_points < MIN_FOLLOWED_POINTS) needToInit = true; 
}

void station::calcOptFlow()
{
	if (!kp.prev_points.empty())
	{
		if (prevGray.empty()) gray.copyTo(prevGray);

		calcOpticalFlowPyrLK(prevGray, gray, /*prev*/ kp.prev_points, /*next*/ kp.current_points,
			kp.status, kp.err, winSize, 5, termcrit, 0, 0.001);

		//cout << "calc " << timeSec << " sec " << "  " << points[1].size() << endl;

		Affine = estimateRigidTransform(kp.prev_points, kp.current_points, true);
		//Was ich eigentlich tue

//		cout << "Affine: " << Affine.at<double>(0, 2) << " - " << Affine.at<double>(1, 2) << endl << endl;
	}
}

void station::transform_Affine()
{
	if (!Affine.empty() && !kp.prev_points.empty())
	{
		// Affine_x_last = Affine_x;

		//Affine_x = Affine.at<double>(0, 2); //tx von Affinematrix row, col
											// umrechnen feautures
	//	transform(kp.prev_points, kp.calculated_points[0], Affine);

	}
}

void station::draw_aim_point()
{
	if (number_aim_point >= 0 && number_aim_point <= kp.current_points.size()) //TODO manchmal nummer out of size

		circle(image, (Point)kp.current_points[number_aim_point], 16, Scalar(0, 0, 255), 3);

	circle(image, (Point)AimPoint, 16, Scalar(0, 255, 0), 3);
}

void station::draw_prev_points()
{
	for (Point2f p : kp.prev_points)
			circle(image, (Point)p, 3, Scalar(255, 0, 255));

}

void station::draw_current_points()
{
	int i = 0;

	for (Point2f p : kp.current_points)
	{

		if (kp.err[i] > 30)
			circle(image, (Point)p, 8, Scalar(0, 0, 200));
		else
		if (kp.status[i] == 0)

			circle(image, (Point)p, 8, Scalar(200, 0, 0));
			else
			circle(image, (Point)p, 3, Scalar(255, 250, 0));


		i++;
	}

	int m = 0;

	for (int n : kp.numbers_of_downpoints)
	{
		stringstream text;

		text.width(5);
		text.precision(4);

		text << kp.step_length[m];

		//if (kp.current_points[n].x < 1240)
		//{
		//	putText(image, text.str(), kp.current_points[n], FONT_HERSHEY_PLAIN, 1.0f, Scalar(0, 0, 0));
		//}

		m++;
	}

	for (size_t i = 0; i < kp.same_step_pt.size(); i++)
	{
		circle(image, (Point)kp.current_points[kp.same_step_pt[i]], 10, Scalar(0, 0, 160));
	}

}

//void station::draw_calculated_points()
//{
//	for (size_t i = 0; i < kp.calculated_points[0].size(); i++) // TODO was?
//	{
//		circle(image, (Point)kp.calculated_points[0][i], 4, Scalar(0, 0, 250));
//	}
//}

void station::draw_main_points()
{
	for (size_t i = 0; i < kp.background_points.size(); i++) 
	{
		circle(image, (Point)kp.current_points[kp.background_points[i]], 7, Scalar(0, 200, 0));
	}
}

void station::draw_summ_vector()
{
	Point2f p1, p2, p3;
	p2 = fokus;

	while (!kp.summ_queue_empty())
	{
		p3 = p2 + magnify_vektor_draw * kp.get_next_summ_vector();

		line(image, (Point)p2, (Point)p3, Scalar(0, 0, 200), 3);
		circle(image, (Point)p2, 3, Scalar(0, 255, 0), 1);
		p2 = p3;
	};
}

int station::draw_image()
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

/*
	if (kp.summ_vector.size() == step_butch) // wenn anzahl frames wird erreicht dann abbilden 
	{

		frame_time = kp.get_queue_time(); //Zeit für Frame holen

		draw_summ_vector();



		//	draw_calculated_points();

		time = 0; // und time 0 stop und warte auf tastatur

	}
	*/
	time = 0;

	return time;
}

void station::draw_step_vectors() // batch
{
	Point2f p0, p1;
	p1 = Point2f(0, 0);

	for (int i = 0; i < kp.current_points.size(); i++)
	{

		p0 = kp.current_points[i];

		while (!kp.step_vector_empty(i))
		{
			//p1 = p0 + magnif * kp.get_next_step_vector(i); //HACK entnahme aus queue vector
			p1 = p0 + magnify_vektor_draw * kp.get_next_step_vector(i); //HACK entnahme aus queue vector

			if (kp.status[i] == 1 && kp.err[i] < 10 )
				line(image, (Point)p0, (Point)(p1), Scalar(0, 250, 0));
			else
				line(image, (Point)p0, (Point)(p1), Scalar(0, 0, 250));
			//circle(image, (Point)p0, 2, Scalar(0, 255, 0), 1);

			p0 = p1;
		};




	}

}

void station::draw_nearest_point()

{
	float* move = new float[kp.MAX_COUNT];

	float dist = 0.0f;
	float max_move = 0.0f;
	int neahrest_point = 0;
	bool danger = false;

}

void station::show_image()
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

	setWindowTitle("LK Demo", text.str());

	imshow("LK Demo", image);

}

void station::calculate_move_vectors()
{
	kp.calculate_move_vectors();
	main_of_frame = kp.summ_vector.back();
	//cout << "main: " << main.x << " - " << main.y << endl << endl;
}

void station::swap()
{
	kp.swap();

	cv::swap(prevGray, gray);
}

bool station::key(int wait)
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

	//case 'k':
	//	cam_calibrate();
	}

	return false;
}
/*
void station::look_to_aim()
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
		needToInit = true;
	}



}
*/
int station::find_nearest_point(Point2f pt)
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

int station::collect_step_vectors()
{

	int number_followed_points = kp.save_step_vectors();
	return 0;
}
// Bearbeitet Frame in schritten
bool station::proceed_frame(Mat* frame, std::vector <keypoints_flow>* key_points)
{

	take_picture(frame);

	imshow("LK Demo", *frame);
	/*
	if (needToInit)
	{
		find_keypoints();
		needToInit = false;
		kp.swap();
		return false; // flow wird nicht gesucht
	}

	calcOptFlow();

	// punkte ersetzen
	*/
	kp.clear();


	for (keypoints_flow p : *key_points)
	{
		kp.err.push_back(5.0);
		kp.status.push_back(1);
		kp.current_points.push_back(p.p);
		kp.prev_points.push_back(p.p - p.flow);
	}


	// check_for_followed_points();

//	kp.calc_distances();

	//	check_for_followed_points(); //TODO zuerst finden die Punkte die gut sind (status) 
	//nur dann collect step vectors.

	collect_step_vectors();

//	transform_Affine();

	calculate_move_vectors();

	int wait_time = draw_image();

	show_image();

//	look_to_aim();

	if (key(wait_time)) return true;

	return false;
}

void station::new_data_proceed(UDP_Base* udp_base)
{



	//udp_base->udp_data_received();

	//send antwort an server 
}


