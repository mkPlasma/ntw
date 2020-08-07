#pragma once

/*
 *	physStruct.h
 *
 *	Structs for use in physicsEngine.h.
 *
 */

#include"objects/object.h"

class Portal;


// Pair of objects for collisions
struct ObjectPair{
	Object* object1;
	Object* object2;

	bool operator==(const ObjectPair& a) const{
		return (object1 == a.object1 && object2 == a.object2) ||
			(object1 == a.object2 && object2 == a.object1);
	}

	// Hash for unordered_map
	size_t operator()(const ObjectPair& a) const{
		return reinterpret_cast<size_t>(object1) + reinterpret_cast<size_t>(object2);
	}
};


// Info on found separating axis for reuse next frame
struct SATSeparatingAxis{
	int index1;
	int index2;
	bool isEdgePair;
};


// Store distance and index of queried faces/edges
struct SATContactInfo{
	float distance;
	bool isEdgePair;
	int index1;
	int index2;
};


// Cache results of SAT collision tests
struct SATCollisionInfo{
	bool updated;
	bool collided;
	SATSeparatingAxis separatingAxis;
	SATContactInfo contactInfo;

	SATCollisionInfo() : updated(true) {}
};


// Single contact between two objects
struct Contact{
	Vec3 normal;
	Vec3 tangent1;
	Vec3 tangent2;

	Vec3 obj1ContactGlobal;
	Vec3 obj2ContactGlobal;

	Vec3 obj1ContactVector;
	Vec3 obj2ContactVector;

	float depth;
	float closingSpeed;

	float lambdaSum;
	float lambdaSumTan1;
	float lambdaSumTan2;

	// For sound
	float lambdaAvg;
	float lambdaAvgTan1;
	float lambdaAvgTan2;
	int numSolves;

	bool valid;
	bool isNew;

	Contact() : depth(0), closingSpeed(0), lambdaSum(0), lambdaSumTan1(0), lambdaSumTan2(0),
		lambdaAvg(0), lambdaAvgTan1(0), lambdaAvgTan2(0), numSolves(0), valid(true), isNew(true) {}
};


// Group of contacts between two objects
struct ContactManifold{
	ObjectPair objects;
	vector<Contact> contacts;
	float maxDistance;
	bool checked;

	ContactManifold() : checked(false) {}
};


// Info on collisions given to objects
struct ObjectContactInfo{
	Object* object;
	Vec3 normal;
};


// Object/Portal pair for portal collisions
struct ObjectPortalPair{
	Object* object;
	Portal* portal;

	bool operator==(const ObjectPortalPair& a) const{
		return object == a.object && portal == a.portal;
	}

	// Hash for unordered_map
	size_t operator()(const ObjectPortalPair& a) const{
		return reinterpret_cast<size_t>(object) + reinterpret_cast<size_t>(portal);
	}
};


// Current portal collision info
struct PortalCollisionInfo{
	ObjectPortalPair objectPortalPair;
	bool objectInFront;
	bool updated;
	bool withPlayer;
};
