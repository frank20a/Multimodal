#include "PointSph.h"

using namespace std;

vec PointSph::to_cartesian() {
	return vec(cos(theta) * sin(phi) * rho, sin(theta) * sin(phi) * rho, cos(phi) * rho);
}

void PointSph::from_cartesian(vec p) {
	rho = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	theta = atan2(p.y, p.x);
	phi = acos(p.z / rho);
}

void PointSph::from_spherical(double rho, double theta, double phi) {
	PointSph::rho = rho;
	PointSph::theta = theta;
	PointSph::phi = phi;
}