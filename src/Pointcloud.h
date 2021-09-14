#pragma once

#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <string.h>
#include <MathGeoLib.h> 
#include <iostream>
#include <fstream>
#include <cstring>
#include <random>
#include <set>
#include "Mesh.h"

// Floor recognition constants
#define FLOOR_Z_OFFSET 0.02
#define FLOOR_RANSAC_REPETITIONS 250
#define FLOOR_RANSAC_THRESHOLD 0.2
// DBSCAN constants
#define DBSCAN_E 1.4
#define DBSCAN_MIN_PTS 20
// Clustering constants
#define RECOGNITION_THRESHOLD 0.75
#define WEIGHT_DISTANCE 0.7
#define WEIGHT_VOLUME 0.05
#define WEIGHT_POINTS 0.1
#define WEIGHT_HEIGHT 0.15

// Helper functions
double point_plane_dist(vec P, vec* plane);
double dist(vec P1, vec P2);
std::vector<vec> load_pointcloud(std::string filepath);
void bb(std::vector<vec> pt_cloud, vec* res);
double score(std::vector<vec> obj, std::vector<vec> prototype);
vec get_center(std::vector<vec> obj);
void recenter(std::vector<vec>& obj);
double volume(vec* bb);

// Flags
static bool AXIS = true;
static bool FLOOR_METHOD = false;

class Pointcloud : public vvr::Scene {
public:
	Pointcloud();
	//~Pointcloud();

	// Overrides
	void keyEvent(unsigned char key, bool up, int modif) override;
	void mouseWheel(int dir, int modif) override;



	// Public methods
	const char* getName() const { return "Pointcloud Scene"; }
	void reload();
	void detect_floor(bool method = Z_THRESH);
	void cluster();
	void classify();
	void test_triangulation();

	// Public members
	const enum { Z_THRESH = false, RANSAC = true };

private:
	// Private methods
	void draw() override;
	void reset() override;

	// Private members
	std::string filename;											// Source
	std::vector<vec> cloud, floor;									// Pointclouds	
	std::vector<std::vector<vec>> clusters;							// Clusters pointclouds
	std::vector<int> classes;										// Clustering results
	bool floor_seperated, triangulated, clusterized, classified;	// Flags
	Mesh_ temp_mesh;												// Triangulated mesh
	static const unsigned int colour_num = 10;						// Number of colours for class representation
	const vvr::Colour colours[colour_num] = {						// Colours for class representation
		vvr::Colour::green,
		vvr::Colour::white,
		vvr::Colour::red,
		vvr::Colour::grey,
		vvr::Colour::yellow,
		vvr::Colour::magenta,
		vvr::Colour::cyan,
		vvr::Colour::orange,
		vvr::Colour::yellowGreen,
		vvr::Colour::darkGreen
	};
	const unsigned int col = rand() % colour_num;					// Colour seed
	std::vector<std::vector<vec>> prototypes = {					// Prototype clouds
		load_pointcloud("../prototype/car_00064_2257.bin"),
		load_pointcloud("../prototype/car_01143_2317.bin"),
		load_pointcloud("../prototype/pedestrian_00231_1938.bin"),
		load_pointcloud("../prototype/pedestrian_01126_2023.bin")
	};
	const enum { CAR1 = 0, CAR2, PEDESTRIAN1, PEDESTIAN2 };
};