#include "Scene3D.h"

Scene3D::Scene3D(int a, int b) {

	reset();
}

void Scene3D::reset() {
	Scene::reset();
	
	
}

void Scene3D::mousePressed(int x, int y, int modif) {
	Scene::mousePressed(x, y, modif);
	
	
}

void Scene3D::keyEvent(unsigned char key, bool up, int modif) {
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key) {
		case 's': 
			break;
	}
}

void Scene3D::draw() {
	
}