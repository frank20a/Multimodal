#pragma once

#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h> 

float dist2D(vec p1, vec p2);
bool inside_circle2D(Circle c, vec p);
bool inside_circumcircle2D(math::Triangle tr, vec p);
bool violates_Delaunay(std::vector<vec> pcl, math::Triangle tr);
bool equal_lines(vec pa1, vec pa2, vec pb1, vec pb2);
bool tr_common_side(math::Triangle a, math::Triangle b);
bool tr_common_point(math::Triangle a, math::Triangle b);
bool equal_point2D(vec p1, vec p2);
Circle find_circumcircle2D(vec p1, vec p2, vec p3);
Circle find_circumcircle2D(math::Triangle tr);

class Mesh_ : public vvr::Mesh{
public:
	// Constructors
	Mesh_() : vvr::Mesh::Mesh() { triangulated = true; }
	Mesh_(const std::string& objFile, const std::string& texFile = std::string(), bool ccw = true)
		: vvr::Mesh::Mesh(objFile, texFile, ccw) { triangulated = true; }
	Mesh_(const vvr::Mesh& original) : vvr::Mesh::Mesh(original) { triangulated = true; }
	Mesh_(std::vector<vec> pcl) : vvr::Mesh::Mesh() {
		triangulate(pcl);
	}

	// Public methods

	// Public members

private:
	// Private methods
	void triangulate(std::vector<vec> pcl);

	// Private members
	bool triangulated = false;
};

