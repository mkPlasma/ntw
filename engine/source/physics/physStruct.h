#pragma once

/*
 *	physStruct.h
 *
 *	Structs for use in physicsEngine.h.
 *
 */

#include"objects/object.h"
#include"objects/physicsObject.h"


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


struct Contact{
	Vec3 penetration;
	Vec3 tangent1;
	Vec3 tangent2;

	Vec3 obj1ContactGlobal;
	Vec3 obj2ContactGlobal;

	Vec3 obj1ContactLocal;
	Vec3 obj2ContactLocal;

	Vec3 obj1ContactVector;
	Vec3 obj2ContactVector;


	float closingSpeed;

	float lambdaSum;
	float lambdaSumTan1;
	float lambdaSumTan2;

	bool persistent;

	Contact() : closingSpeed(0), lambdaSum(0), lambdaSumTan1(0), lambdaSumTan2(0), persistent(false) {}
};


struct ContactManifold{
	ObjectPair objects;
	vector<Contact> contacts;
	bool checked;

	ContactManifold() : checked(false) {}
};
