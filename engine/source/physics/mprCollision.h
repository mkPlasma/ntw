#pragma once

/*
 *	simplex.h
 *
 *	MPRCollision object for GJK collision detection.
 *
 */

#include"objects/object.h"
#include"physStruct.h"


// Minkowski difference vertex with individual object vertices
struct CVertex{
	Vec3 v;
	Vec3 vObj1;
	Vec3 vObj2;

	bool operator==(const CVertex& a){
		return v == a.v;
	}
};

// EPA polytope geometry
struct EPAFace{
	CVertex v1;
	CVertex v2;
	CVertex v3;

	Vec3 normal;
	float distance;

	EPAFace() : distance(0) {}

	// Priority queue compare
	bool operator()(const EPAFace& a, const EPAFace& b){
		return a.distance < b.distance;
	}
};

struct EPAEdge{
	CVertex v1;
	CVertex v2;
};


class MPRCollision{

	const vector<Vec3> cubeVerts_ = {
		Vec3(1, 1, 1),
		Vec3(1, 1, -1),
		Vec3(1, -1, 1),
		Vec3(1, -1, -1),
		Vec3(-1, 1, 1),
		Vec3(-1, 1, -1),
		Vec3(-1, -1, 1),
		Vec3(-1, -1, -1),
	};

	const Vec3 zero_ = Vec3();

	// Objects
	Object* object1_;
	Object* object2_;

	Vec3 obj1Pos_;
	Vec3 obj2Pos_;

	Matrix obj1Rotation_;
	Matrix obj2Rotation_;


	// Cached transformed hitbox vertices
	vector<Vec3> object1Verts_;
	vector<Vec3> object2Verts_;

	// Vertices of the simplex
	vector<CVertex> vertices_;

	// Direction to add support point in
	Vec3 direction_;
	
	// EPA polytope geometry
	vector<EPAFace> faces_;
	vector<EPAEdge> edges_;

	
	void init();

	bool addSupportPoint();
	CVertex getSupportPoint();
	Vec3 getFurthestPoint(bool useObject1, const Vec3& direction);

	void addFace(const CVertex& v1, const CVertex& v2, const CVertex& v3);
	void addEdge(const CVertex& v1, const CVertex& v2);
	int getFaceClosestToOrigin();

	void setContactInfo(Contact& c);

public:
	MPRCollision(Object* object1, Object* object2);

	// Test collision, true if objects are colliding
	bool testCollision();

	// Get penetration vector and object contact points
	Contact getContact();
};
