
#ifndef _ARM
#include "targetver.h"

#endif

#include <stdio.h>
//#include <tchar.h>

#include <iostream>
#include <fstream>

#include <opencv2/core/utility.hpp>
#include "opencv2/video.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"



using namespace cv;
using namespace std;

inline bool isFlowCorrect(Point2f u)
{
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

static Vec3b computeColor(float fx, float fy)
{
    static bool first = true;

    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static Vec3i colorWheel[NCOLS];

    if (first)
    {
        int k = 0;

        for (int i = 0; i < RY; ++i, ++k)
            colorWheel[k] = Vec3i(255, 255 * i / RY, 0);

        for (int i = 0; i < YG; ++i, ++k)
            colorWheel[k] = Vec3i(255 - 255 * i / YG, 255, 0);

        for (int i = 0; i < GC; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255, 255 * i / GC);

        for (int i = 0; i < CB; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255 - 255 * i / CB, 255);

        for (int i = 0; i < BM; ++i, ++k)
            colorWheel[k] = Vec3i(255 * i / BM, 0, 255);

        for (int i = 0; i < MR; ++i, ++k)
            colorWheel[k] = Vec3i(255, 0, 255 - 255 * i / MR);

        first = false;
    }

    const float rad = sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float)CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    Vec3b pix;

    for (int b = 0; b < 3; b++)
    {
        const float col0 = colorWheel[k0][b] / 255.f;
        const float col1 = colorWheel[k1][b] / 255.f;

        float col = (1 - f) * col0 + f * col1;

        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= .75; // out of range

        pix[2 - b] = static_cast<uchar>(255.f * col);
    }

    return pix;
}

static void drawOpticalFlow(const Mat_<Point2f>& flow, Mat& dst, float maxmotion = -1)
{
    dst.create(flow.size(), CV_8UC3);
    dst.setTo(Scalar::all(0));

    // determine motion range:
    float maxrad = maxmotion;

    if (maxmotion <= 0)
    {
        maxrad = 1;
        for (int y = 0; y < flow.rows; ++y)
        {
            for (int x = 0; x < flow.cols; ++x)
            {
                Point2f u = flow(y, x);

                if (!isFlowCorrect(u))
                    continue;

                maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
            }
        }
    }

    for (int y = 0; y < flow.rows; ++y)
    {
        for (int x = 0; x < flow.cols; ++x)
        {
            Point2f u = flow(y, x);

            if (isFlowCorrect(u))
                dst.at<Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
        }
    }
}

static void help()
{
	std::cout << "\nThis program demonstrates\n"
		<< "Usage: \n" << "demhist [videoe_name -- Defaults to ../data/baboon.jpg]" << std::endl;
}

const char* keys =
{
	"{help h||}{@image|../data/baboon.jpg|input image file}"
};

int main(int argc, const char* argv[])
{
	CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		help();
		return 0;
	}
	string inputVideo = parser.get<string>(0);


	VideoCapture cam(0);
	
	if (!cam.isOpened()) {
		//error in opening the video input
		cerr << "Unable to open video: " << endl;
		exit(EXIT_FAILURE);
	}
 
	Mat frame0, frame1, fg0, fg1;
	string file = "test.b";
	Mat_<Point2f> flow;

	std::vector<cv::Point2f> features_prev, features_next, features_calc;
	std::vector<uchar> status;
	std::vector<float> err;

	//vector<Point2f> fea_calc;

	//BFMatcher matcher(NORM_L2);
	//matcher.match(descriptors1, descriptors2, matches);

	//Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();

	
	//Ptr<ORB> dt = ORB::create();
	//Ptr<BRISK> dt1 = BRISK::create(30);

	//dt->setThreshold(3e-4);
	//dt->;

	Ptr<Feature2D> detector;
	Mat first_desc;

	vector<KeyPoint> first_kp;
	vector<Point2f> object_bb;

	float qLevel = 0.05f;
	float minDist = 6.0f;

	//detector = dt1;

	cam >> frame0;
	cvtColor(frame0, fg0, COLOR_BGR2GRAY);

	//detector->detectAndCompute(fg0, noArray(), first_kp, first_desc);

	cv::goodFeaturesToTrack(fg0, // the image 
		features_prev,   // the output detected features
		380,  // the maximum number of features 
		qLevel,     // quality level
		minDist     // min distance between two features
		);

	bool fr; 

	for (int i = 0; fr = cam.read(frame1); i++)
	{

		//frame1.convertTo(fg1, CV_8UC1);
		cvtColor(frame1, fg1, COLOR_BGR2GRAY);

		

		const double start = (double)getTickCount();

		cv::goodFeaturesToTrack(fg1, // the image 
			features_prev,   // the output detected features
			380,  // the maximum number of features 
			qLevel,     // quality level
			minDist     // min distance between two features
			);

		features_calc.resize(features_prev.size());

		calcOpticalFlowPyrLK(fg0, fg1, features_prev, features_calc, status, err);


		const double timeSec = (getTickCount() - start) / getTickFrequency();

		// features_next = features_prev;

		cout << "calc" << timeSec << " sec " << "  " << features_prev.size() << endl;

		Mat Affine = estimateRigidTransform(features_prev, features_calc, true);

		// detector->detectAndCompute(fg0, noArray(), first_kp, first_desc);

		//tvl1->calc(fg0, fg1, flow);

		cout << Affine << endl;

		Mat out;
		Mat calc(features_prev);

		if (!Affine.empty())
		{
			// umrechnen features
			perspectiveTransform(features_prev, calc, Affine);

			cout << features_prev.size() << " - " << calc.cols  << endl;


			// umrechnen feautures
			transform(features_prev, calc, Affine);

			// Umrechnen bild in neue lage
			// warpAffine(fg0, out, Affine, fg0.size());
		}


		//Mat substruct = abs(fg1 - out);

		//Mat substruct1 = abs(fg1 - fg0);

		//convertieren in Point2f
		vector<Point2f> fea_calc = Mat_<Point2f>(calc.reshape(1, calc.cols*calc.rows));
		
		frame1.copyTo(out);

		
		
		//for (size_t i = 0; i < first_kp.size(); i++)
		//{
		//	KeyPoint k = first_kp[i];
		//	circle(out, Point((int)k.pt.x, (int)k.pt.y), 4, Scalar(0, 0, 250));
		//}

		for (size_t i = 0; i < features_prev.size(); i++)
		{
			Point2f p = features_prev[i];
			//alle gefundene features
			circle(out, Point((int)p.x, (int)p.y), 4, Scalar(0, 0, 250));
		}

		for (size_t i = 0; i < fea_calc.size(); i++)
		{
			Point2f p = fea_calc[i];
			// draw berechnete features
			circle(out, Point((int)p.x, (int)p.y), 6, Scalar(255, 0, 0));
		}

		

		
		
		for (int index = 0; index < features_prev.size() && index < features_calc.size(); index++)
		{

			if (status[index])
			{
			Point2f p = features_prev[index];

				// draw letzte gefundene FEAtures
				//circle(out, Point((int)p.x, (int)p.y), 2, Scalar(255, 250, 0));

				// draw linie von vorherigen Stand zu letzten features
				line(out, Point((int)p.x, (int)p.y), Point((int)features_calc[index].x, (int)features_calc[index].y), Scalar(0, 250, 255));
			}

		
		}

		
		

		drawOpticalFlow(flow, out);

		
		imshow("keys", out);
		

		fg1.copyTo(fg0);

		waitKey(20);
	}

	waitKey();
    return 0;
}
