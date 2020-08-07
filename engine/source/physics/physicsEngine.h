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

	World& world_;

	// Object lists
	vector<Object*>& objects_;
	vector<PhysicsObject*>& dynamicObjects_;

	AABBTree aabbTree_;

	vector<ContactManifold> contactManifolds_;

	vector<Constraint> constraints_;
	vector<ContactConstraint> contactConstraints_;

	unordered_map<ObjectPair, SATCollisionInfo, ObjectPair> satCollisions_;
	unordered_map<ObjectPortalPair, PortalCollisionInfo, ObjectPortalPair> portalCollisions_;


	void checkCollisions();
	void resolveCollision(const AABBPair& pair);
	void resolvePortalCollision(Object* object, Portal* portal);

public:
	PhysicsEngine(World& world, vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects);

	void update();

	void cleanup();


	vector<Object*> castRay(const Vec3& position, const Vec3& direction, float maxDistance);

	void addObject(Object* object);
	void removeObject(Object* object);

	void addPortal(Portal* portal);
	void removePortal(Portal* portal);
	
	const vector<ContactManifold>& getContactManifolds();
	const unordered_map<ObjectPortalPair, PortalCollisionInfo, ObjectPortalPair>& getPortalCollisions();
};
