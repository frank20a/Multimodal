#include "Mesh.h"
#include <algorithm>

using namespace std;

bool triangle_equality(Triangle a, Triangle b) {
    return (equal_point2D(a.a, b.a) || equal_point2D(a.a, b.b) || equal_point2D(a.a, b.c)) &&
        (equal_point2D(a.b, b.a) || equal_point2D(a.b, b.b) || equal_point2D(a.b, b.c)) &&
        (equal_point2D(a.c, b.a) || equal_point2D(a.c, b.b) || equal_point2D(a.c, b.c));
}

bool pseudo_sorter(Triangle a, Triangle b) {
    return dist2D(vec(0,0,0), find_circumcircle2D(a).pos) < dist2D(vec(0, 0, 0), find_circumcircle2D(b).pos);
}

// Bowyer-Watson algorithm
void Mesh_::triangulate(std::vector<vec> pcl) {
	if (triangulated) return;

    vector<Triangle> triangles;

    int NN = pcl.size(), nn = 0;                                                        // Counters
    Triangle super = Triangle(vec(0, 100, 0), vec(0, -100, 100), vec(0, -100, -100));   // Create super triangle
    triangles.push_back(super);                                                         // And push it

    for (auto& p : pcl) {

        //cout << ++nn << "/" << NN << " -> " << triangles.size() << endl;      // DEBUG

        // Find Delaunay violations
        vector<int> rem;            // Keep indexes of triangles to delete
        int N = triangles.size();   // Set initial triangle set size
        for (int i = 0; i < N; i++) {
            if (inside_circumcircle2D(triangles[i], p)) {
                rem.push_back(i);   // Remember to delete super-triangle
                // Add triangles that form from super-triangle vertices and the point
                triangles.push_back(Triangle(p, triangles[i].b, triangles[i].c));
                triangles.push_back(Triangle(triangles[i].a, p, triangles[i].c));
                triangles.push_back(Triangle(triangles[i].a, triangles[i].b, p));
            }
        }
        // Delete old super-triangles
        N = rem.size() - 1;
        for (int i = N; i >= 0; i--) {
            triangles.erase(triangles.begin() + rem[i]);
        }
        rem.clear();

        // Delete duplicates
        sort(triangles.begin(), triangles.end(), pseudo_sorter);                    // Use pseudo sorting func to bring duplicate triangles together
        auto it = unique(triangles.begin(), triangles.end(), triangle_equality);    // Remove CONSECUTIVE duplicates
        triangles.resize(distance(triangles.begin(), it));

    }

    // Remove all the vertices of the super triangle and the triangles containing them
    int N = triangles.size() - 1;
    for (int i = N; i >= 0; i--) {
        if (tr_common_point(super, triangles[i])) {
            //cout << "Erased " << i << endl;       // DEBUG
            triangles.erase(triangles.begin() + i);
        }
    }
    
    // Fix the actual Mesh (xd vvr go brrrr)
    getVertices().clear();
    getTriangles().clear();
    for (auto i = 0; i < triangles.size(); i++) {
        getVertices().push_back(triangles[i].a);
        getVertices().push_back(triangles[i].b);
        getVertices().push_back(triangles[i].c);
    }
    update();
    for (auto i = 0; i < triangles.size(); i++) 
        getTriangles().push_back(vvr::Triangle(&getVertices(), 3 * i, 3 * i + 1, 3 * i + 2));



	triangulated = true;
}

Circle find_circumcircle2D(vec p1, vec p2, vec p3) {
    Circle res;

    float A, B, C, D;
    A = p1.y * (p2.z - p3.z) - p1.z * (p2.y - p3.y) + p2.y * p3.z - p3.y * p2.z;
    B = (pow(p1.y, 2) + pow(p1.z, 2)) * (p3.z - p2.z) + (pow(p2.y, 2) + pow(p2.z, 2)) * (p1.z - p3.z) + (pow(p3.y, 2) + pow(p3.z, 2)) * (p2.z - p1.z);
    C = (pow(p1.y, 2) + pow(p1.z, 2)) * (p2.y - p3.y) + (pow(p2.y, 2) + pow(p2.z, 2)) * (p3.y - p1.y) + (pow(p3.y, 2) + pow(p3.z, 2)) * (p1.y - p2.y);
    D = (pow(p1.y, 2) + pow(p1.z, 2)) * (p3.y * p2.z - p2.y * p3.z) + (pow(p2.y, 2) + pow(p2.z, 2)) * (p1.y * p3.z - p3.y * p1.z) + (pow(p3.y, 2) + pow(p3.z, 2)) * (p2.y * p1.z - p1.y * p2.z);

    res.pos = vec(0.0, -B / (2.0 * A), -C / (2.0 * A));
    res.r = pow((pow(B, 2) + pow(C, 2) - 4 * A * D) / (4 * pow(A, 2)), .5);
    //cout << "(" << res.GetCentre().x << "," << res.GetCentre().y << ") R" << res.GetRadius() << endl;
    return res;
}
Circle find_circumcircle2D(Triangle tr) {
    return find_circumcircle2D(tr.a, tr.b, tr.c);
}

bool inside_circle2D(Circle c, vec p) {
    return dist2D(c.pos, p) < c.r;
}

bool inside_circumcircle2D(Triangle tr, vec p) {
    return inside_circle2D(find_circumcircle2D(tr), p);
}

float dist2D(vec p1, vec p2) {
    return pow(pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2), 0.5);
}

bool violates_Delaunay(std::vector<vec> pcl, Triangle tr) {
    Circle circumcircle = find_circumcircle2D(tr);

    for (auto& p : pcl) {
        if (inside_circle2D(circumcircle, p)) return true;
    }

    return false;
}

bool equal_lines(vec pa1, vec pa2, vec pb1, vec pb2) {
    return dist2D(pa1, pb1) + dist2D(pa2, pb2) < 0.1 || dist2D(pa1, pb2) + dist2D(pa2, pb1) < 0.1;
}

bool tr_common_side(Triangle a, Triangle b) {
    return equal_lines(a.a, a.b, b.a, b.b) || equal_lines(a.a, a.b, b.a, b.c) || equal_lines(a.a, a.b, b.b, b.c) ||
        equal_lines(a.a, a.c, b.a, b.b) || equal_lines(a.a, a.c, b.a, b.c) || equal_lines(a.a, a.c, b.b, b.c) ||
        equal_lines(a.c, a.b, b.a, b.b) || equal_lines(a.c, a.b, b.a, b.c) || equal_lines(a.c, a.b, b.b, b.c);
}

bool tr_common_point(Triangle a, Triangle b) {
    return equal_point2D(a.a, b.a) || equal_point2D(a.a, b.b) || equal_point2D(a.a, b.c) ||
        equal_point2D(a.b, b.a) || equal_point2D(a.b, b.b) || equal_point2D(a.b, b.c) ||
        equal_point2D(a.c, b.a) || equal_point2D(a.c, b.b) || equal_point2D(a.c, b.c);
}

bool equal_point2D(vec p1, vec p2) {
    return dist2D(p1, p2) < 0.1;
}