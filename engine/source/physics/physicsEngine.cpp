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
		if((*i)->getPhysicsType() == PHYS_STATIC)
			staticObjects_.push_back(*i);
		else if((*i)->getPhysicsType() == PHYS_SEMI_DYNAMIC)
			semiDynamicObjects_.push_back(*i);
	}

	initAABBCollisions();
}

void PhysicsEngine::update(){

	// Apply initial updates
	for(auto i = dynamicObjects_.begin(); i != dynamicObjects_.end(); i++){

		PhysicsObject* obj = *i;

		// Gravity
		obj->addVelocity(Vec3(0, 0, -0.3f));

		obj->tUpdate();
	}

	// Collision detection (in physicsEngineCollisions.cpp)
	checkCollisions();

	// Solve constraints, repeating until all constraints are satisfied
	bool solved;
	int iter = 0;

	do{
		// Solve and apply constraints
		for(auto i = contactConstraints_.begin(); i != contactConstraints_.end(); i++)
			(*i).solve();

		for(auto i = constraints_.begin(); i != constraints_.end(); i++)
			(*i).solve();

		// Check that constraints have been solved
		solved = true;
		for(auto i = contactConstraints_.begin(); i != contactConstraints_.end(); i++){

			// This constraint not solved, so solve all constraints again
			if(!(*i).calcConstraint()){
				solved = false;
				break;
			}
		}
		for(auto i = constraints_.begin(); i != constraints_.end(); i++){
			if(!(*i).calcConstraint()){
				solved = false;
				break;
			}
		}

		iter++;

	} while(!solved && iter < PHYS_MAX_CONSTRAINT_ITER);

	// Update all objects
	for(auto i = dynamicObjects_.begin(); i != dynamicObjects_.end(); i++)
		(*i)->update();
}

vector<ContactManifold>& PhysicsEngine::getCollisions(){
	return contactManifolds_;
}
