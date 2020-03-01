#pragma once

/*
 *	physicsEngine.h
 *
 *	Simulates object physics and collisions.
 *
 */

#include"objects/object.h"
#include"objects/physicsObject.h"
#include"physStruct.h"
#include"constraints/contactConstraint.h"
#include<unordered_map>

using std::unordered_map;


class PhysicsEngine{

	const Quaternion noRotation = Quaternion();

	// Object lists
	vector<Object*>& objects_;
	vector<Object*> staticObjects_;
	vector<Object*> semiDynamicObjects_;
	vector<PhysicsObject*>& dynamicObjects_;

	// Collisions
	vector<Vec3> cubeVerts_;
	vector<vector<CollisionInterval>> aabbCollisionIntervals_;
	unordered_map<ObjectPair, int, ObjectPair> aabbCollisions_;

	vector<ContactManifold> contactManifolds_;

	vector<Constraint> constraints_;
	vector<ContactConstraint> contactConstraints_;


	void initAABBCollisions();

	void checkCollisions();

	void updateAABBCollisionIntervals();
	vector<float> getCollisionInterval(Object* obj);

	bool checkContactValidity(ObjectPair& objects, Contact& c);
	void findOptimalContacts(ContactManifold& m);


public:
	PhysicsEngine(vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects);

	void init();

	void update();

	vector<ContactManifold>& getCollisions();
};
