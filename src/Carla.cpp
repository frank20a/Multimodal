#include "Carla.h"

using namespace std;
using namespace cv;

Carla::Carla(string filename) : img("../Carla_images/", filename) {
	cout << "Constructed Carla " << filename << endl;
}

Carla::~Carla() {
	destroyAllWindows();
	cout << "Destroyed Carla " << img.get_filename() << endl;
}

bool Carla::reload() {
	return img.reload();
}

void Carla::process_image(double rho, double theta, int thresh, double minLen, double maxGap) {
	img.bw();
	img.filter(5);
	img.Canny(200, 250, 5);
	img.Hough(rho, theta, thresh, minLen, maxGap);
}
void Carla::process_image(int win, int thresh1, int thresh2) {
	img.bw();
	img.filter(win);
	img.Canny(thresh1, thresh2, 5);
	img.Hough();
}
void Carla::process_image() {
	process_image(5, 200, 250);
}

void Carla::show(bool stop) { img.show(stop); }
void Carla::show() { img.show(); }

void Carla::finalize() { img.finalize(); }

// Get methods
Frame Carla::get_frame() { return img; }