#pragma once

/*
 *	satCollision.h
 *
 *	SAT collision tester and contact point generator.
 *
 */

class SATCollision;

#include"objects/object.h"
#include"physStruct.h"

// Temp include for CVertex struct
#include"physics/mprCollision.h"

// Threshold to allow for a small amount of penetration before registering collision
#define NTW_SAT_THRESHOLD	-0.0001f

// Directional bias for resolving collisions in a particular direction
// For example, force SAT to use bottom face of player hitbox to ensure ramps work
#define NTW_SAT_NORMAL_BIAS	0


// Store distance and index of queried faces/edges
struct SATContactInfo{
	float distance;
	bool isEdgePair;
	int index1;
	int index2;
};

// Projected interval for edge query
struct SATInterval{
	float v1;
	float v2;
};


class SATCollision{

	// Objects
	Object* object1_;
	Object* object2_;


	// Cached transformed hitbox vertices
	const Hitbox& obj1Hitbox_;
	const Hitbox& obj2Hitbox_;

	// Store info for contact point generation
	SATContactInfo contactInfo_;


	float queryFaces(const Hitbox& hitbox1, const Hitbox& hitbox2, bool useIndex1);
	float queryEdges(const Hitbox& hitbox1, const Hitbox& hitbox2);

	Vec3 getSupportPoint(const Hitbox& hitbox, const Vec3& direction);

	float getFaceToPointDistance(const SATFace& f, const Vec3& v);

	SATInterval project(const Hitbox& hitbox, const Vec3& axis);

	bool isMinkowskiFace(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);
	float getEdgeToEdgeDistance(const SATHalfEdge& e1, const SATHalfEdge& e2);


	vector<Vec3> clipFaces(const SATFace& f1, const SATFace& f2);

	bool setContactInfo(Contact& c);

public:
	SATCollision(Object* object1, Object* object2);

	// Test collision, true if objects are colliding
	bool testCollision();

	// Get penetration vector and object contact points
	ContactManifold getContactPoints();
};
