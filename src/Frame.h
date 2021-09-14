#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class Frame {
public:
	Frame(std::string folder, std::string filename);
	Frame();
	~Frame();

	// Show the picture
	void show(std::string win_name, bool stop = true, std::string txt = "");
	void show(bool stop, std::string txt = "") { show(filename, stop, txt); };
	void show() { show(filename); };

	// Image processing functions
	void bw();
	void filter(int sizex, int sizey);
	void filter(int size = 3);
	void Canny(double thresh1 = 200, double thresh2 = 250, int apertureSize = 5);
	void Hough(double rho = 1.0, double theta = CV_PI / 180, int thresh = 175, double minLen = 60.0, double maxGap = 6.0);
	void finalize();

	// Other public methods
	bool reload();

	// Get methods
	cv::Mat get_img() { return img; };
	std::string get_filename() { return filename; };
	std::vector<cv::Vec4i> get_houghLines() { return hough_lines; };

	// Set methods
	void set_file(std::string folder, std::string filename);
	void set_houghLines(std::vector<cv::Vec4i> lines) { hough_lines = lines; };

	// Copy methods
	void copy_houghLines(Frame frame) { set_houghLines(frame.get_houghLines()); };

private:
	// Private methods
	void pause(char cr);
	void pause();

	// Private members
	cv::Mat img;
	std::string filename;
	std::string filepath;
	bool canny;
	std::vector<cv::Vec4i> hough_lines;
};

