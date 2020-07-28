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


// Threshold to allow for a small amount of penetration before registering collision
#define NTW_SAT_THRESHOLD	-0.0001f


class SATCollision{

	// Info on found separating axis for reuse next frame
	struct SeparatingAxis{
		int index1;
		int index2;
		bool isFace;
	};

	// Store distance and index of queried faces/edges
	struct ContactInfo{
		float distance;
		bool isEdgePair;
		int index1;
		int index2;
	};

	// Projected interval for edge query
	struct EdgeInterval{
		float v1;
		float v2;
	};


	// Colliders
	const Collider* collider1_;
	const Collider* collider2_;

	// Transformed hitbox of each collider
	const Hitbox& hitbox1_;
	const Hitbox& hitbox2_;

	// Store info of separating axis
	SeparatingAxis separatingAxis_;

	// Store info for contact point generation
	ContactInfo contactInfo_;


	float queryFaces(const Hitbox& hitbox1, const Hitbox& hitbox2, bool useIndex1);
	float queryEdges(const Hitbox& hitbox1, const Hitbox& hitbox2);

	Vec3 getSupportPoint(const Hitbox& hitbox, const Vec3& direction);

	EdgeInterval project(const Hitbox& hitbox, const Vec3& axis);

	bool isMinkowskiFace(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);

	bool setContactInfo(Contact& c);

public:
	SATCollision(const Collider* collider1, const Collider* collider2);

	// Test collision, returns true if objects are colliding
	bool testCollision();
	bool testCollision(const SeparatingAxis& axis);

	// Get penetration vector and object contact points
	ContactManifold getContactPoints();
};
