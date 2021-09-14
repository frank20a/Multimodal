#pragma once

#include <MathGeoLib.h> 
#include <iostream>

class PointSph {
public:
	// Constructors
	PointSph() { from_spherical(0, 0, 0); };
	PointSph(vec p) { from_cartesian(p); };
	PointSph(double rho, double theta, double phi) { from_spherical(rho, theta, phi); };

	// Get methods
	vec to_cartesian();

	// Set methods
	void from_cartesian(vec p);
	void from_spherical(double rho, double theta, double phi);

	// Public members
	double rho, theta, phi;

	// Operators
	PointSph operator+(PointSph& p) {
		return PointSph(to_cartesian() + p.to_cartesian());
	}
	PointSph operator-(PointSph& p) {
		return PointSph(to_cartesian() - p.to_cartesian());
	}
	PointSph operator*(const double& a) {
		return PointSph(this->rho * a, this->theta, this->rho);
	}
	PointSph operator/(const double& a) {
		return PointSph(this->rho / a, this->theta, this->rho);
	}
	void operator+=(PointSph& p) {
		PointSph tmp (to_cartesian() + p.to_cartesian());
		this->rho = tmp.rho; this->theta = tmp.theta; this->phi = tmp.phi;
	}
	void operator-=(PointSph& p) {
		PointSph tmp(to_cartesian() - p.to_cartesian());
		this->rho = tmp.rho; this->theta = tmp.theta; this->phi = tmp.phi;
	}
	void operator*=(const double& a) {
		this->rho *= a;
	}
	void operator/=(const double& a) {
		this->rho /= a;
	}
};

