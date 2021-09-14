#include "Pointcloud.h"
#include "PointSph.h"

using namespace std;
using namespace vvr;

Pointcloud::Pointcloud() {
	vvr::Shape::DEF_LINE_WIDTH = 2;
	vvr::Shape::DEF_POINT_SIZE = 4;
	m_bg_col = vvr::Colour::black;
	m_perspective_proj = true;
	m_fullscreen = false;

	cout << "Pointcloud filename: ";
	//cin >> Pointcloud::filename;
	Pointcloud::filename = "pointcloud_00500_.bin";
	reset();

	cout << "Loaded pointcloud " << filename;
}

void Pointcloud::reload() {
	cloud.clear();
	floor.clear();
	clusters.clear();
	classes.clear();
	floor_seperated = clusterized = triangulated = classified = false;

	//for (auto& prototype : prototypes) recenter(prototype);

	cloud = load_pointcloud("../Lidar/" + filename);
}

void Pointcloud::reset() {
	Scene::reset();
	reload();
}

void Pointcloud::keyEvent(unsigned char key, bool up, int modif) {
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key) {
	case 'r':
		reset();
		break;
	case 'f':
		detect_floor(FLOOR_METHOD);
		break;
	case 'm':
		FLOOR_METHOD = !FLOOR_METHOD;
		cout << "Floor detection method: " << (FLOOR_METHOD ? "RANSAC" : "Z Threshold") << endl;
		break;
	case 'c':
		cluster();
		break;
	case 'd':
		cout << "Bounding box colours: Green: Car, Red: Pedestrian\n";
		classify();
		break;
	case 'a':
		AXIS = !AXIS;
		break;
	case 't':
		test_triangulation();
		break;
	default:
		return;
	}
}

void Pointcloud::mouseWheel(int dir, int modif) {
	Scene::mouseWheel(dir, modif);
}

void Pointcloud::draw() {
	// Draw axis
	if (AXIS) {
		LineSeg3D(0, 0, 0, 100, 0, 0, Colour::blue).draw();		// X-Axis
		LineSeg3D(0, 0, 0, 0, 100, 0, Colour::green).draw();	// Y-Axis
		LineSeg3D(0, 0, 0, 0, 0, 100, Colour::red).draw();		// Z-Axis
	}

	// Draw clusters w/ different colours
	unsigned int c = col;
	for (auto& cluster : clusters) {
		for (auto& point : cluster) {
			Point3D(point.x, point.y, point.z, colours[c % colour_num]).draw();
		}
		c++;
	}

	// Draw bounding boxes for classified objects (Green: Car, Red: Pedestrian)
	for (unsigned int i = 0; i < classes.size(); i++) {
		if (classes[i] >= 0) {
			vec bb_pts[2];
			bb(clusters[i], bb_pts);

			LineSeg3D(bb_pts[0].x, bb_pts[0].y, bb_pts[0].z, bb_pts[1].x, bb_pts[0].y, bb_pts[0].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[0].y, bb_pts[0].z, bb_pts[0].x, bb_pts[0].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[0].y, bb_pts[1].z, bb_pts[0].x, bb_pts[1].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[0].y, bb_pts[1].z, bb_pts[1].x, bb_pts[0].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[1].x, bb_pts[0].y, bb_pts[0].z, bb_pts[1].x, bb_pts[0].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[0].y, bb_pts[0].z, bb_pts[0].x, bb_pts[1].y, bb_pts[0].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[1].x, bb_pts[0].y, bb_pts[0].z, bb_pts[1].x, bb_pts[1].y, bb_pts[0].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[1].y, bb_pts[0].z, bb_pts[1].x, bb_pts[1].y, bb_pts[0].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[1].x, bb_pts[0].y, bb_pts[1].z, bb_pts[1].x, bb_pts[1].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[1].y, bb_pts[0].z, bb_pts[0].x, bb_pts[1].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[1].x, bb_pts[1].y, bb_pts[0].z, bb_pts[1].x, bb_pts[1].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();

			LineSeg3D(bb_pts[0].x, bb_pts[1].y, bb_pts[1].z, bb_pts[1].x, bb_pts[1].y, bb_pts[1].z,
				(classes[i] ? vvr::Colour::red : vvr::Colour::green)).draw();
		}
	}

	// Draw pointcloud
	for (auto& point : cloud)
		Point3D(point.x, point.y, point.z, vvr::Colour::darkOrange).draw();

	// Draw floor
	for (auto& point : floor)
		Point3D(point.x, point.y, point.z, vvr::Colour::blue).draw();

	if (triangulated) {
		for (auto& pt : prototypes[2]) Point3D(pt.x, pt.y, pt.z, Colour::white).draw();
		temp_mesh.draw(Colour::green, SOLID);
		temp_mesh.draw(Colour::black, WIRE);
		//temp_mesh.draw(Colour::black, NORMALS);
		//temp_mesh.draw(Colour::black, AXES);
	}
}

void Pointcloud::detect_floor(bool method) {
	if (floor_seperated) throw "Floor already detected!";
	cout << "Started floor seperation...\n";

	vector<vec> temp = cloud;
	cloud.clear();

	if (method) {	// RANSAC
		unsigned int best_score = 0;

		for (int i = 0; i < FLOOR_RANSAC_REPETITIONS; i++) {
			vector<vec> inlies, outlies;

			// Get 3 random seed points (plane)
			int seed[3];
			do {
				for (int i = 0; i < 3; i++) seed[i] = rand() % temp.size();
			} while (seed[0] == seed[1] || seed[1] == seed[2] || seed[0] == seed[2]);
			vec seed_pts[3] = { temp[seed[0]], temp[seed[1]], temp[seed[2]] };

			// Evaluate current seed
			unsigned int score = 0;
			for (auto& p : temp) {
				if (point_plane_dist(p, seed_pts) < FLOOR_RANSAC_THRESHOLD) {
					inlies.push_back(p);
					score++;
				}
				else outlies.push_back(p);
			}

			// Override results if current repetition is best
			if (score > best_score) {
				best_score = score;
				cloud = outlies;
				floor = inlies;
			}
		}
	}
	else {			// Z-Threshold
		// Find upper/lower limits
		double min = temp[0].z, max = temp[0].z;
		for (auto& p : temp) {
			if (p.z < min) min = p.z;
			if (p.z > max) max = p.z;
		}

		// Keep lower 2% as floor
		for (auto& p : temp) {
			if (p.z >= min + FLOOR_Z_OFFSET * (max - min)) cloud.push_back(p);
			else floor.push_back(p);
		}
	}

	floor_seperated = true;
}

void Pointcloud::cluster() {
	if (!floor_seperated) throw "Floor not yet seperated";
	if (clusterized) throw "Data already clustered!";
	cout << "Starting clustering...\n";

	// DBSCAN
	set<int> checked;

	for (int i = 0; i < cloud.size(); i++) {		// Iterated non-floor points

		if (checked.count(i) > 0) continue;			// Skip checked points

		checked.insert(i);							// Add current to checked points
		set<int> temp = { i };						// Create cluster
		vector<int> fringe = { i };					// Create fringe

		 do {
			// Get current point and remove it from fringe
			vec curr = cloud[fringe.back()];
			fringe.pop_back();

			// Iterate current testing cluster
			for (unsigned int j = 0; j < cloud.size(); j++) {
				if (checked.count(j) || temp.count(j)) continue;

				// If point is not already checked and satisfies DBSCAN epsilon insert it in fringe and cluster
				if (dist(curr, cloud[j]) < DBSCAN_E) {
					temp.insert(j);
					fringe.push_back(j);
				}
			}
		 } while (!fringe.empty() && temp.size() >= DBSCAN_MIN_PTS);
		
		// Check if current cluster satisfies DBSCAN minimum points 
		if (temp.size() >= DBSCAN_MIN_PTS) {
			// Convert cluster from index to vec
			vector<vec> cluster;
			for (auto& index : temp) {
				cluster.push_back(cloud[index]);
				checked.insert(index);
			}

			// Push cluster to clusters set
			clusters.push_back(cluster);
			draw();
		}
	}

	cloud.clear();
	clusterized = true;
}

void Pointcloud::classify() {
	if (!clusterized) throw "Data not clusterized yet!";
	if (classified) throw "Data already classified!";
	cout << "Starting classification...\n";

	for (auto& cluster : clusters) {
		double best_score = -1;
		int best_prototype;

		for (int i = 0; i < 4; i++) {
			double s = score(cluster, prototypes[i]);
			if (s > best_score) {
				best_score = s;
				best_prototype = i;
			}
		}

		if (best_score > RECOGNITION_THRESHOLD) {
			classes.push_back((int)(best_prototype / 2));
			cout << ((int)(best_prototype / 2) ? "Pedestrian " : "Car        ") << best_score * 100 << "%" << endl;
		}
		else classes.push_back(-1);
	}

	classified = true;
}

void Pointcloud::test_triangulation() {
	cout << "Testing triangulation...\n";

	// Rotate
	PointSph avg(get_center(prototypes[2]));
	for (auto& pt : prototypes[2]) {
		PointSph tmp(pt);
		tmp.theta -= avg.theta;

		pt = tmp.to_cartesian();
	}

	vec avg1(get_center(prototypes[2]));
	for (auto& pt : prototypes[2]) {
		pt.x -= avg1.x;
	}

	temp_mesh = Mesh_(prototypes[2]);
	triangulated = true;
}

// Helper functions

double point_plane_dist(vec P, vec* plane) {
	vec A = plane[0], B = plane[1], C = plane[2];

	float a, b, c, d;

	// Find plane coeffs
	a = (B.y - A.y) * (C.z - A.z) - (C.y - A.y) * (B.z - A.z);
	b = (B.z - A.z) * (C.x - A.x) - (C.z - A.z) * (B.x - A.x);
	c = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
	d = -(a * A.x + b * A.y + c * A.z);
	
	// Calculate and return distance
	return abs(a * P.x + b * P.y + c * P.z + d) / sqrt(a * a + b * b + c * c);
}

double dist(vec P1, vec P2) {
	return pow(pow(P2.x - P1.x, 2.0) + pow(P2.y - P1.y, 2.0) + pow(P2.z - P1.z, 2.0), 0.5);
}

vector<vec> load_pointcloud(std::string pclFile) {
	vector<vec> cloud;
	vec cm(0, 0, 0);

	fstream input(pclFile.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Could not read file: " << pclFile << endl;
		exit(EXIT_FAILURE);
	}

	input.seekg(0, ios::beg);
	for (int i = 0; input.good() && !input.eof(); i++) {
		vec point;
		float intensity;
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&intensity, sizeof(float));
		cm += point;
		cloud.push_back(point);
	}
	input.close();

	return cloud;
}

void bb(vector<vec> pt_cloud, vec* res) {
	res[0] = vec(-9999999, -9999999, -9999999);
	res[1] = vec( 9999999,  9999999,  9999999);

	for (auto& pt : pt_cloud) {
		// Set higher bb point
		if (pt.x > res[0].x) res[0].x = pt.x;
		if (pt.y > res[0].y) res[0].y = pt.y;
		if (pt.z > res[0].z) res[0].z = pt.z;
		// Set lower bb point
		if (pt.x < res[1].x) res[1].x = pt.x;
		if (pt.y < res[1].y) res[1].y = pt.y;
		if (pt.z < res[1].z) res[1].z = pt.z;
	}
}

double score(vector<vec> obj, vector<vec> prototype) {
	recenter(obj);
	recenter(prototype);

	double distance_error = 0, volume_error = 0, points_error = 0, height_error = 0;

	// ==== Calculate distance error ====
	for (auto& pt1 : obj) {

		//double best = 999999;
		double best[3] = { 999999 };							// Using 3-NN
		for (auto& pt2 : prototype) {

			double d = exp(pow(dist(pt1, pt2), 2));				// Use exponential MSE to reward similarity/punish non-similarity
			if (d < best[0]) {
				best[2] = best[1];
				best[1] = best[0];
				best[0] = d;
			}
			//if (d < best) best = d;
		}

		distance_error += (best[0] + best[1] + best[2]) / 3;	// Average 3-NN scores
		//distance_error += best;
	}
	distance_error = obj.size() / distance_error;

	// ==== Calculate volume error ====
	vec bbs[4];
	bb(obj, bbs);
	bb(prototype, bbs + 2);
	//volume_error = exp(-pow(abs(volume(bbs) - volume(bbs + 2)), 1.0/3.0) / 3.0);
	volume_error = 1 - (abs(volume(bbs + 2) - volume(bbs)) / max(volume(bbs + 2), volume(bbs)));

	// ==== Calculate points error ====
	points_error = 1 - (abs((double)obj.size() - prototype.size()) / max(obj.size(), prototype.size()));

	// ==== Calculate height error ====
	double h1 = abs(bbs[0].z - bbs[1].z), h2 = abs(bbs[2].z - bbs[3].z);
	height_error = 1 - (abs(h1 - h2) / max(h1, h2));

	// Average cloud score and return
	return (distance_error * WEIGHT_DISTANCE + volume_error * WEIGHT_VOLUME + points_error * WEIGHT_POINTS + height_error * WEIGHT_HEIGHT) 
		/ (WEIGHT_DISTANCE + WEIGHT_VOLUME + WEIGHT_POINTS + WEIGHT_HEIGHT);
}

vec get_center(vector<vec> obj) {
	vec avg(0, 0, 0);
	for (auto& pt : obj) avg += pt;
	return avg / obj.size();
}

void recenter(vector<vec>& obj) {
	// Recenter obj
	vec avg = get_center(obj);
	for (auto& pt : obj) pt -= avg;
}

double volume(vec* bb) {
	return abs((bb[1].x - bb[0].x) * (bb[1].y - bb[0].y) * (bb[1].z - bb[0].z));
}