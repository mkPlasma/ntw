#pragma once

/*
 *	physStruct.h
 *
 *	Structs for use in physicsEngine.h.
 *
 */

#include"objects/object.h"


// Interval for AABB collisions
struct CollisionInterval{
	float position;
	bool start;
	Object* object;

	bool operator<(const CollisionInterval& a) const{
		return (position < a.position) || (position == a.position && object == a.object && start && !a.start);
	}

	bool operator<=(const CollisionInterval& a) const{
		return position <= a.position;
	}

	bool operator==(const CollisionInterval& a) const{
		return position == a.position;
	}
};


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


// Single contact between two objects
struct Contact{
	Vec3 normal;
	Vec3 tangent1;
	Vec3 tangent2;

	Vec3 obj1ContactGlobal;
	Vec3 obj2ContactGlobal;

	// Obsolete for SAT collisions
	Vec3 obj1ContactLocal;
	Vec3 obj2ContactLocal;

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
struct ContactInfo{
	Object* object;
	Vec3 normal;
};
