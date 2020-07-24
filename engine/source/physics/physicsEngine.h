#pragma once

/*
 *	physicsEngine.h
 *
 *	Simulates object physics and collisions.
 *
 */

class PhysicsEngine;

#include"objects/object.h"
#include"objects/physicsObject.h"
#include"physics/physStruct.h"
#include"physics/aabbTree.h"
#include"constraints/contactConstraint.h"
#include<unordered_map>

using std::unordered_map;

class World;


class PhysicsEngine{

	bool initialized_;

	World& world_;

	// Object lists
	vector<Object*>& objects_;
	vector<PhysicsObject*>& dynamicObjects_;

	// Collisions
	vector<vector<CollisionInterval>> aabbCollisionIntervals_;
	unordered_map<ObjectPair, int, ObjectPair> aabbCollisions_;

	AABBTree aabbTree_;

	vector<ContactManifold> contactManifolds_;

	vector<Constraint> constraints_;
	vector<ContactConstraint> contactConstraints_;


	void initAABBCollisions();

	void checkCollisions(float timeDelta, bool fullUpdate);
	void resolveCollision(const ObjectPair& objects, float timeDelta);

	void updateAABBCollisionIntervals();
	vector<float> getCollisionInterval(Object* obj);


public:
	PhysicsEngine(World& world, vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects);

	void init();

	void update(float timeDelta, bool fullUpdate);


	vector<Object*> castRay(const Vec3& position, const Vec3& direction, float maxDistance);

	void initObject(Object* object);
	void removeObject(Object* object);

	bool isInitialized();
	vector<ContactManifold>& getContactManifolds();

	AABBTree::Node* getRoot();
};
