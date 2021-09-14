#include "Frame.h"

using namespace std;
using namespace cv;

Frame::Frame(string folder, string filename) {
	set_file(folder, filename);
	if (!reload()) throw invalid_argument("File not found!");
}
Frame::Frame() {}

Frame::~Frame() {
	//cout << "Destroyed Frame " << filename << endl;
}

void Frame::set_file(string folder, string filename) {
	Frame::filename = filename;
	filepath = folder + filename;
}

bool Frame::reload() {
	img = imread(filepath, IMREAD_COLOR);
	canny = false;

	if (img.empty()) {
		//cout << "Image couldn't be loaded from " << filepath << endl;
		return false;
	}

	//cout << "Loaded " << filename << endl;
	return true;
}

void Frame::show(std::string win_name, bool stop, std::string txt) {
	Mat tmp = Frame::img.clone();
	putText(tmp, txt, Point(10, 17), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1.2);

	imshow(win_name, tmp);

	// pause
	if (stop) cv::waitKey(0);
}

void Frame::bw() {
	cvtColor(img, img, cv::COLOR_BGR2GRAY);
	//cout << filename << " converted to grayscale." << endl;
}


void Frame::filter(int sizex, int sizey) {
	GaussianBlur(img, img, Size(sizex, sizey), 0);
	medianBlur(img, img, max(sizex, sizey));

	//cout << filename << " filtered w/ " << sizex << "x" << sizey << "px Gaussian and " << max(sizex, sizey) << "px Median." << endl;
}
void Frame::filter(int size) { filter(size, size); }

void Frame::Canny(double thresh1, double thresh2, int apertureSize) {
	cv::Canny(img, img, thresh1, thresh2, apertureSize);
	//cout << filename << " processed w/ Canny method thresh1=" << thresh1 << ", thresh2=" << thresh2 << " and aperture set to " << apertureSize << "." << endl;
	canny = true;
}


void Frame::Hough(double rho, double theta, int thresh, double minLen, double maxGap) {
	if (!canny) throw "Hough before Canny!";

	HoughLinesP(img, hough_lines, rho, theta, thresh, minLen, maxGap);

	cvtColor(img, img, COLOR_GRAY2BGR);
	for (auto & line : hough_lines)
		cv::line(img, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 3, LINE_AA);
}

void Frame::finalize() {
	reload();
	for (auto& line : hough_lines)
		cv::line(img, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(0, 0, 255), 3, LINE_AA);
}