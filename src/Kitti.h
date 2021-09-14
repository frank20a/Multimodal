#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "Frame.h"

class Kitti {
public:
	Kitti(std::string testcase, unsigned int fps, bool rgb = false);
	~Kitti();

	// Public methods
	void reload();
	void play(int camera);
	void process_video(int win, int thresh1, int thresh2);
	void process_video();
	void finalize();

	// Class members
	const enum { left = 0, right, left_rgb, right_rgb };
	const std::string camera_name[4] = { "Left", "Right", "Left RGB", "Right RGB" };

private:
	// Private methods

	// Private members
	std::vector <Frame> videos[4];
	std::string folderpath;
	std::string testcase;
	unsigned int fps;
	bool rgb;
};

