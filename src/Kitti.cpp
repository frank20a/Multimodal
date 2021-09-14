#include "Kitti.h"

using namespace std;
using namespace cv;

Kitti::Kitti(std::string testcase, unsigned int fps, bool rgb) {
	Kitti::fps = fps;
	Kitti::testcase = testcase;
	Kitti::rgb = rgb;
	folderpath = "../Kitti_images/" + testcase;
	folderpath += "/";

	reload();

	cout << "Constructed Kitti " << testcase << endl;
}

Kitti::~Kitti() {
	destroyAllWindows();
	cout << "Destroyed Kitti " << testcase << endl;
}

void Kitti::reload() {
	Frame tmp;

	cout << "Loading frames";

	for (int j = 0; j < 4; j++) {
		string folder = folderpath + "image_0";
		folder += to_string(j);
		folder += "/data/";

		for (int i = 0; true; i++) {

			// Create filename
			string file = to_string(i + 1);
			int size_of_num = file.size();
			for (int k = 0; k < 10 - size_of_num; k++) file = "0" + file;
			file += ".png";

			// Load image
			tmp.set_file(folder, file);
			if (!tmp.reload()) break;

			// Append image to proper video
			videos[j].push_back(tmp);

			tmp.show("Loading...", false, "LOADING Frame " + to_string(i + 1) + " from image_0" + to_string(j));
			waitKey(1);
		}

		cout << ".";
	}
	cout << endl;
	destroyAllWindows();
}

void Kitti::play(int camera) {
	bool loop = true;
	while (loop) {
		int c = 0;
		for (auto& frame : videos[camera]) {
			string win_name = testcase + " - ";
			win_name += Kitti::camera_name[camera];

			frame.show(win_name, false, "Frame " + to_string(++c) + "/" + to_string(videos[camera].size()));

			char c = (char)waitKey(1000 / fps);
			if (c == 27) {
				destroyAllWindows();
				loop = false;
				break;
			}
		}
	}
}

void Kitti::process_video(int win, int thresh1, int thresh2) {
	cout << "Processing videos";

	for (int i = 0; i < 2; i++) {
		int c = 0;
		for (auto & frame : videos[i]) {
			frame.bw();
			frame.filter(win);
			frame.Canny(thresh1, thresh2, 3);
			frame.Hough(1, CV_PI / 180, 75, 60, 6);

			videos[i + 2][c].copy_houghLines(frame);

			frame.show("Processing...", false, "PROCESSING " + to_string(i+1) + "/2 Frame " + to_string(++c) + "/" + to_string(videos[i].size()));
			waitKey(1);
		}

		cout << ".";
	}
	cout << endl;
	destroyAllWindows();
}
void Kitti::process_video() {
	process_video(5, 200, 250);
}

void Kitti::finalize() {
	for (int i = 0; i < 4; i++) {
		int c = 0;
		for (auto& frame : videos[i]) {
			frame.finalize();
			frame.show("Finalizing...", false, "FINALIZING Frame " + to_string(++c) + "/" + to_string(videos[i].size()));
			waitKey(1);
		}
	}
	destroyAllWindows();
}