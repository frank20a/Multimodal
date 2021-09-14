#pragma once

#include <string>
#include "Frame.h"

class Carla {
public:
	Carla(std::string filename);
	~Carla();

	// Public methods
	bool reload();
	void process_image(double rho, double theta, int thresh, double minLen, double maxGap);
	void process_image(int win, int thresh1, int thresh2);
	void process_image();
	void show(bool stop);
	void show();
	void finalize();

	// Get methods
	Frame get_frame();


private:
	// Private methods

	// Private members
	Frame img;
};