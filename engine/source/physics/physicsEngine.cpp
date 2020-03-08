#include"physicsEngine.h"

#include"physDefine.h"
#include"core/error.h"
#include<algorithm>

using std::max;


PhysicsEngine::PhysicsEngine(vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects)
	: objects_(objects), dynamicObjects_(physicsObjects) {

	// Cube hitbox shape
	cubeVerts_ = {
		Vec3(1, 1, 1),
		Vec3(-1, 1, 1),
		Vec3(1, -1, 1),
		Vec3(1, 1, -1),
	};

	// AABB collision interval lists
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
}

void PhysicsEngine::init(){

	// Organize objects
	staticObjects_.clear();
	semiDynamicObjects_.clear();

	for(auto i = objects_.begin(); i != objects_.end(); i++){
		if((*i)->getPhysicsType() == PhysicsType::STATIC)
			staticObjects_.push_back(*i);
		else if((*i)->getPhysicsType() == PhysicsType::SEMI_DYNAMIC)
			semiDynamicObjects_.push_back(*i);
	}

	initAABBCollisions();
}

void PhysicsEngine::update(){

	// Apply initial updates
	for(auto i = dynamicObjects_.begin(); i != dynamicObjects_.end(); i++){

		PhysicsObject* obj = *i;

		// Gravity
		if(obj->useGravity())
			obj->addVelocity(Vec3(0, 0, -0.3f));

		obj->tUpdatePhysics();
	}

	// Collision detection (in physicsEngineCollisions.cpp)
	checkCollisions();

	// Solve constraints, repeating until all constraints are satisfied
	int iter = 0;

	do{
		bool solved = true;

		// Solve and apply constraints
		for(auto i = contactConstraints_.begin(); i != contactConstraints_.end(); i++)
			solved &= (*i).solve();

		for(auto i = constraints_.begin(); i != constraints_.end(); i++)
			solved &= (*i).solve();

		iter++;

		if(solved)
			break;

		// Temp update all dynamic objects again
		//for(auto i = dynamicObjects_.begin(); i != dynamicObjects_.end(); i++)
		//	((PhysicsObject*)*i)->tUpdatePhysics();

	} while(iter < PHYS_MAX_CONSTRAINT_ITER);


	// Update all objects
	for(auto i = dynamicObjects_.begin(); i != dynamicObjects_.end(); i++)
		(*i)->updatePhysics();
}

vector<ContactManifold>& PhysicsEngine::getContactManifolds(){
	return contactManifolds_;
}
