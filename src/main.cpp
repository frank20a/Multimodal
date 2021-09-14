#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "Carla.h"
#include "Kitti.h"
#include "Pointcloud.h"

using namespace std;

int main(int argc, char* argv[]) {
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

	string filename;

	cout << "Press:\n  1) Carla\n  2) Kitti\n  3) Lidar\nINPUT: ";
	char cmd = cin.get();
	if (cmd == '1') {
		cin >> filename;

		Carla test(filename);
		test.process_image();
		test.show();
		test.finalize();
		test.show();
		return 0;
	}
	else if (cmd == '2') {
		cin >> filename;

		Kitti test1(filename, 10);
		test1.play(Kitti::left_rgb);
		test1.process_video();
		test1.play(Kitti::left);
		test1.finalize();
		test1.play(Kitti::left_rgb);
		return 0;
	}
	else if (cmd == '3') {
		return vvr::mainLoop(argc, argv, new Pointcloud);
	}
	else return -1;

}