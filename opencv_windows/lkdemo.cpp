#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <string.h>

#include "SerialPort.h"

using namespace cv;
using namespace std;

#pragma warning(disable : 4996)


static void help()
{
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
            "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
            "\tn - switch the \"night\" mode on/off\n"
            "To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
    if( event == EVENT_LBUTTONDOWN )
    {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
}

int main( int argc, char** argv )
{
    
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 500;
    bool needToInit = false;
    bool nightMode = false;
	double timeSec;

	const char* portName = "\\\\.\\COM5";
	char m[40];
	
	SerialPort sp(portName);
	Sleep(500);
	sp.writeSerialPort("#1P1800T1300\r\n");
	//Sleep(2000);
	while (sp.readSerialPort(m, 2) != 2);
		sp.writeSerialPort("#1P800T6000\r\n");
	while (sp.readSerialPort(m, 2) != 2);
		sp.writeSerialPort("#1P1800T60\r\n");
	sp.~SerialPort();

    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");

	VideoCapture cap;

	if (input.size() == 1 && isdigit(input[0]))
		cap.open(0);
		//cap.open(0); //(input[0] - '0');
    else
        cap.open(input);



    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    namedWindow( "LK Demo", 1 );
    setMouseCallback( "LK Demo", onMouse, 0 );

    Mat gray, prevGray, image, frame;
    vector<Point2f> points[2];
	vector<Point2f> calc[2];

	float sum_x = 0.0f;


    for(;;)
    {
        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        if( nightMode )
            image = Scalar::all(0);

        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);

			const double start = (double)getTickCount();

			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);

			timeSec = (getTickCount() - start) / getTickFrequency();

			// cout << "calc" << timeSec << " sec " << "  " << points[1].size() << endl;

			Mat Affine = estimateRigidTransform(points[0], points[1], true);


			if (!Affine.empty())
			{
				//cout << points[0].size() << " - " << calc[0].size() << endl;


				// umrechnen feautures
				transform(points[0], calc[0], Affine);

			}

			calc[1].resize(points[1].size());

			sum_x = 0.0f;

			for (size_t i = 0; i < min(points[1].size(), points[0].size()); i++)
			{
				Point2f p = calc[0][i];
				// draw berechnete features
				circle(image, Point((int)p.x, (int)p.y), 6, Scalar(255, 0, 0));

				calc[1][i] = points[1][i] - points[0][i];

				sum_x += calc[1][i].x;

			}

			sum_x /= min(points[1].size(), points[0].size());




            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }

        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;


		stringstream text;

		//text.width(4);
		text.precision(2);
		
		text << "calc " << timeSec*1000 << " ms " << "  " << points[1].size() << "  " << sum_x;

		putText(image, text.str(), Point(100, 100), FONT_HERSHEY_PLAIN, 2.0f, Scalar(0, 0, 0), 2);

		imshow("LK Demo", image);

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c )
        {
        case 'r':
            needToInit = true;
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            break;
        case 'n':
            nightMode = !nightMode;
            break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }

    return 0;
}
