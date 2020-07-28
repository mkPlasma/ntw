#include"physicsEngine.h"

#include"physDefine.h"
#include"core/error.h"
#include"core/world.h"
#include"physics/satCollision.h"
#include"physics/physFunc.h"
#include"objects/modelFunc.h"
#include<algorithm>

using std::max;


PhysicsEngine::PhysicsEngine(World& world, vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects)
	: world_(world), objects_(objects), dynamicObjects_(physicsObjects), initialized_(false) {

}

void PhysicsEngine::init(){

	// Initialize physics objects and add colliders to AABB tree
	for(Object* obj : objects_)
		addObject(obj);

	initialized_ = true;
}

void PhysicsEngine::update(){

	// Apply initial updates
	for(PhysicsObject* obj : dynamicObjects_){

		// Gravity
		if(obj->useGravity())
			obj->addVelocity(obj->getGravityDirection() * 12 * NTW_PHYS_TIME_DELTA);

		obj->tUpdatePhysics();
	}

	// Collision detection (in physicsEngineCollisions.cpp)
	checkCollisions();

	// Solve constraints, repeating until all constraints are satisfied
	int iter = 0;

	do{
		// Solve and apply constraints
		for(ContactConstraint& c : contactConstraints_){
			c.solve();

			// Temp update objects
			if(c.getObjects().object1->getPhysicsType() == PhysicsType::RIGID_BODY)
				((PhysicsObject*)c.getObjects().object1)->tUpdatePhysics();

			if(c.getObjects().object2->getPhysicsType() == PhysicsType::RIGID_BODY)
				((PhysicsObject*)c.getObjects().object2)->tUpdatePhysics();
		}

		for(Constraint& c : constraints_)
			c.solve();

		iter++;

		// Check if all constraints are satisfied
		for(ContactConstraint& c : contactConstraints_)
			if(!c.isSolved())
				goto constraintLoop;

		for(Constraint& c : constraints_)
			if(!c.isSolved())
				goto constraintLoop;

		// Solved, exit loop
		break;

	constraintLoop:;
	} while(iter < NTW_PHYS_MAX_CONSTRAINT_ITER);

	// Update all objects
	for(PhysicsObject* obj : dynamicObjects_)
		obj->updatePhysics();
}

void PhysicsEngine::cleanup(){
	aabbTree_.clear();
	contactManifolds_.clear();
	constraints_.clear();
	contactConstraints_.clear();
}


vector<Object*> PhysicsEngine::castRay(const Vec3& position, const Vec3& direction, float maxDistance){

	// TEMPORARY
	// brute force against dynamic objects
	vector<Object*> colliding;

	for(PhysicsObject* pObj : dynamicObjects_)
		if(ntw::raycast(position, direction, maxDistance, pObj->getColliders()[0].hitboxTransformed) != -1)
			colliding.push_back(pObj);

	return colliding;
}

void PhysicsEngine::addObject(Object* object){

	// Initialize physics
	if(object->getPhysicsType() == PhysicsType::RIGID_BODY || object->getPhysicsType() == PhysicsType::SIMPLE)
		((PhysicsObject*)object)->initPhysics();


	// Add colliders to AABB tree
	const vector<Collider>& colliders = object->getColliders();

	for(const Collider& c : colliders)
		aabbTree_.add(&c);
}

void PhysicsEngine::removeObject(Object* object){

	// Remove colliders from AABB tree
	const vector<Collider>& colliders = object->getColliders();

	for(const Collider& c : colliders)
		aabbTree_.remove(&c);
}

void PhysicsEngine::addPortal(Portal* portal){
	aabbTree_.add(&portal->getCollider());
}

void PhysicsEngine::removePortal(Portal* portal){
	aabbTree_.remove(&portal->getCollider());
}


bool PhysicsEngine::isInitialized(){
	return initialized_;
}

const vector<ContactManifold>& PhysicsEngine::getContactManifolds(){
	return contactManifolds_;
}

const unordered_map<ObjectPortalPair, PortalCollisionInfo, ObjectPortalPair>& PhysicsEngine::getPortalCollisions(){
	return portalCollisions_;
}
