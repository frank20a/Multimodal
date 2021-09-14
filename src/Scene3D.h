#pragma once

#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>

using namespace vvr;

class Scene3D : public vvr::Scene {
public:
	Scene3D(int a, int b);
	virtual void mousePressed(int x, int y, int modif) override;
	void keyEvent(unsigned char key, bool up, int modif) override;

private:
	void draw() override;
	void reset() override;
};