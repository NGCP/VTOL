#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <ctime>
#include <librealsense2/rs.hpp>


using namespace cv;
using namespace std;

//this is involved with putting text on a frame?
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

int main(int argc, char* argv) {
	Mat frame, hsv, lab, YCB, grey;
	Mat filtered_pink, upper_hsv_range;
	bool bSuccess;



	//setting up video capture
	//Open default camera and reads video
	VideoCapture cap(0);

	//use pre-recored video
	//VideoCapture cap("proofOC2.mp4");

	if (cap.isOpened() == false) {
		std::cout << "Cannot open camera" << endl;
		cin.get();
		return -1;
	}

	//manually sets camera dimensions
	cap.set(CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

	//manually sets camera dimensions
	//cap.set(CAP_PROP_FRAME_WIDTH, 16);
	//cap.set(CAP_PROP_FRAME_HEIGHT, 9);

	//finds & prints camera dimensions 
	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);
	std::cout << "Resolution is: " << dWidth << " x " << dHeight << endl;

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
	cap.read(frame);

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
		bSuccess = cap.read(frame);
		if (!bSuccess) {
			//cout << "camera is disconnected" << endl;
			//wait for key press
			cin.get();
			break;
		}
		//BGR (each channel is color and brightness)
		cv::imshow("webcam", frame);
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
		else {
			putText(frame, "Tracking failure detected", Point(100, 80),
				FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
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