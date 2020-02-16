#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <ctime>
#include <librealsense2/rs.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"


using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

//this is involved with putting text on a frame?
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


int main(int argc, char* argv) {
	Mat frame, hsv, lab, YCB, grey, good_matches_mat, live_descriptors, Homo;
	vector<KeyPoint> live_keypoints;
	std::vector<std::vector<cv::DMatch>> knn_matches;
	vector<DMatch> good_matches;
	vector<Point2f> SUGV_vector, live_vector, SUGV_corners(4), live_corners(4);
	Mat filtered_pink, upper_hsv_range;
	bool bSuccess;
	const float ratio_thresh = 0.7f;

	//abstract the device (camera?) 
	rs2::pipeline pipe;
	
	//create config for configuring the pipeline with a non defult profile
	rs2::config cfg;

	//Add desired streams to start streaming with the requested config
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	//or: cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);

	//start stream with requested config
	pipe.start(cfg);

	//let the camera warmup -drop the first couple of frames to let auto-exposure stablize
	rs2::frameset frames;
	for (int i = 0; i < 30; i++) {
		//wait for all configed streams to produce a frame
		frames = pipe.wait_for_frames();
	}

	//load SUGV ref_image
	Mat ref_SUGV = imread("SUGV.jpg");
	imshow("SUGV", ref_SUGV);

	//Object detection by Homography 
	//detect keypoints of SUGV
	int minHessian = 400;
	Ptr<ORB> detector = ORB::create(minHessian);
	vector<KeyPoint> SUGV_keypoints;
	Mat SUGV_descriptors;
	detector->detectAndCompute(ref_SUGV, noArray(), SUGV_keypoints, SUGV_descriptors);

	drawKeypoints(ref_SUGV, SUGV_keypoints, ref_SUGV);
	imshow("keypoints", ref_SUGV);

	//declare matcher
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
	

	
	//list of tracker types
	string trackerTypes[8] = { "BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW",
		"GOTURN", "MOSSE", "CSRT" };

	//create a tracker
	string trackerType = trackerTypes[6];

	Ptr<Tracker> tracker;
	//old pos
	if (trackerType == "BOOSTING")
		tracker = TrackerBoosting::create();
	//better than boost, bad at reporting failure
	if (trackerType == "MIL")
		tracker = TrackerMIL::create();
	//faster than boost and mil, does not handle occlusion well
	if (trackerType == "KCF")
		tracker = TrackerKCF::create();
	//prone to false positives generally pos?
	if (trackerType == "TLD")
		tracker = TrackerTLD::create();
	//good at reporting failures, but dies with too large of a jump in motion, i.e. 
	//fast moving objects
	if (trackerType == "MEDIANFLOW")
		tracker = TrackerMedianFlow::create();
	//DEEP LEARNING requires additional models; pain in the ass?
	if (trackerType == "GOTURN")
		tracker = TrackerGOTURN::create();
	//GOTTA GO FAST GOTTA GO FAST. less accurate than csrt and kcf
	if (trackerType == "MOSSE")
		tracker = TrackerMOSSE::create();
	//more accurate than kcf but a bit slower
	if (trackerType == "CSRT")
		tracker = TrackerCSRT::create();

	//read first frame
	//cap.read(frame);

	//defining inital bounding box. idk what numbers do. are they coords?
	Rect2d bbox(287, 23, 86, 320);

	//select bounding box with mouse
	//bbox = selectROI(frame, false);
	//Display bounding box
	rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);

	//cv::imshow("Tracking", frame);
	tracker->init(frame, bbox);

	//erosion & dilation kernel
	int erosion_size = 6;
	Mat element = getStructuringElement(MORPH_CROSS,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	
	while (true) {
		//get realsense rgb
		rs2::frameset data = pipe.wait_for_frames();
		rs2::frame colour = data.get_color_frame();
		// Query frame size (width and height)
		const int w = colour.as<rs2::video_frame>().get_width();
		const int h = colour.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		cv::Mat frame(cv::Size(w, h), CV_8UC3, (void*)colour.get_data(), cv::Mat::AUTO_STEP);

		imshow("realsense", frame);
		
		//BGR (each channel is color and brightness)
		//cv::imshow("webcam", frame);
		//grab frame from cap before i mess with it in tracking
		cvtColor(frame, hsv, COLOR_BGR2HSV);

		//tick timer
		double timer = (double)getTickCount();

		//update tracking result
		bool ok = tracker->update(frame, bbox);
		float fps = getTickFrequency() / ((double)getTickCount() - timer);
		//if ok, tracking worked, draw the tracked object
		if (ok) {
			rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
		}
		//tracking failure detected
		//when target is lost, re-detect
		else if (!frame.empty()) {
			putText(frame, "Tracking failure detected", Point(100, 80),
				FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);

			//detect
			detector->detectAndCompute(frame, noArray(), live_keypoints, live_descriptors);
			Mat frame2;
			drawKeypoints(frame, live_keypoints, frame2);
			imshow("live_keypoints", frame2);
			
			//match
			matcher->knnMatch(SUGV_descriptors, live_descriptors, knn_matches, 2);
			
			//filter matches with lowe's ratio test
			cout << knn_matches[0][0].distance << endl;
			for (size_t i = 0; i < knn_matches.size(); i++) 
			{
				if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
					good_matches.push_back(knn_matches[i][0]);
				}
			}
			//draw matches
			drawMatches(ref_SUGV, SUGV_keypoints, frame, live_keypoints, good_matches,
				good_matches_mat, Scalar::all(-1), Scalar::all(-1), vector<char>(),
				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			//localize SUGV
			for (size_t i = 0; i < good_matches.size(); i++) {
				//get the keypoints from the good matches
				SUGV_vector.push_back(SUGV_keypoints[good_matches[i].queryIdx].pt);
				live_vector.push_back(live_keypoints[good_matches[i].trainIdx].pt);
			}

			//find homography
			Homo = findHomography(SUGV_vector, live_vector, RANSAC);
			
			//get the corners of the SUGV
			SUGV_corners[0] = Point2f(0, 0);
			SUGV_corners[1] = Point2f((float)ref_SUGV.cols, 0);
			SUGV_corners[2] = Point2f((float)ref_SUGV.cols, (float)ref_SUGV.rows);
			SUGV_corners[3] = Point2f(0, (float)ref_SUGV.rows);
			
			perspectiveTransform(SUGV_corners, live_corners, Homo);

			//draw lines between corners on frame
			line(good_matches_mat, live_corners[0] + Point2f((float)ref_SUGV.cols, 0),
				live_corners[1] + Point2f((float)ref_SUGV.cols, 0), Scalar(0, 255, 0), 4);
			line(good_matches_mat, live_corners[1] + Point2f((float)ref_SUGV.cols, 0),
				live_corners[2] + Point2f((float)ref_SUGV.cols, 0), Scalar(0, 255, 0), 4);
			line(good_matches_mat, live_corners[2] + Point2f((float)ref_SUGV.cols, 0),
				live_corners[3] + Point2f((float)ref_SUGV.cols, 0), Scalar(0, 255, 0), 4);
			line(good_matches_mat, live_corners[3] + Point2f((float)ref_SUGV.cols, 0),
				live_corners[0] + Point2f((float)ref_SUGV.cols, 0), Scalar(0, 255, 0), 4);
			//show matches
			imshow("Homo", good_matches_mat);
		}

		//Display tracker type on frame
		putText(frame, trackerType + "Tracker", Point(100, 20),
			FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

		//Display FPS on frame SSTR broken rn
		//putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50),
			//FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

		//Display tracking frame
		cv::imshow("Tracking", frame);


		//timer
		time_t rawtime;
		struct tm* timeinfo;
		char buffer[80];
		std::time(&rawtime);
		timeinfo = localtime(&rawtime);

		std::strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
		string str333(buffer);


		/*Greyscales
		uses one BGR channel
		*/
		//cvtColor(video, grey, COLOR_BGR2GRAY);
		//imshow("grey", grey);

		/*HSV
		H: hue (Dominant Wavelength
		S: saturation (purity/shades of color)
		V: value (intensity)
		*not as sensitive to lighting varations?
		*/
		

		cv::imshow("hsv_before", hsv);
		cv::inRange(hsv, Scalar(50, 100, 50), Scalar(255, 255, 255), filtered_pink);
		cv::erode(filtered_pink, filtered_pink, element);
		cv::dilate(filtered_pink, filtered_pink, element);
		cv::imshow("hsv", filtered_pink);
		Moments m = moments(filtered_pink, true);
		Point p(m.m10 / m.m00, m.m01 / m.m00);
		//stringstream out;
		//out << "x: " << to_string(p.x) << " y: " << to_string(p.y) << endl;
		//string str_out = out.str();
		//don't use stringstream retard
		//string timer = asctime(localtime(&result));
		//you need endl for python reading
		std::cout << "x: " << to_string(p.x) << " y: " << to_string(p.y) << " time: " << str333 << endl;


		circle(frame, p, 5, Scalar(128, 0, 0), -1);
		imshow("color_tracking", frame);


		//wait for 1 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		int keyValue = waitKey(1);

		if (keyValue == 27)
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
	}
	return 0;
}