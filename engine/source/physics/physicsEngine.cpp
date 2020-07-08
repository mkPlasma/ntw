#include"physicsEngine.h"

#include"physDefine.h"
#include"core/error.h"
#include<algorithm>

using std::max;


PhysicsEngine::PhysicsEngine(vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects)
	: objects_(objects), dynamicObjects_(physicsObjects) {

	// Cube hitbox shape
	/*
	cubeVerts_ = {
		Vec3(1, 1, 1),
		Vec3(-1, 1, 1),
		Vec3(1, -1, 1),
		Vec3(1, 1, -1),
	};
	*/

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

		// Add to lists
		if((*i)->getPhysicsType() == PhysicsType::STATIC)
			staticObjects_.push_back(*i);

		else if((*i)->getPhysicsType() == PhysicsType::SEMI_DYNAMIC)
			semiDynamicObjects_.push_back(*i);

		else if((*i)->getPhysicsType() == PhysicsType::DYNAMIC || (*i)->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE)
			((PhysicsObject*)*i)->initPhysics();
	}

	initAABBCollisions();
}

void PhysicsEngine::update(float timeDelta, bool fullUpdate){

	// Apply initial updates
	for(PhysicsObject* obj : dynamicObjects_){

		bool dynamic = obj->getPhysicsType() == PhysicsType::DYNAMIC;

		// Gravity
		if(obj->useGravity() && (!dynamic || (dynamic && fullUpdate)))
			obj->addVelocity(Vec3(0, 0, -12 * (dynamic ? NTW_PHYS_TIME_DELTA : timeDelta)));

		obj->tUpdatePhysics(timeDelta);
	}

	// Collision detection (in physicsEngineCollisions.cpp)
	checkCollisions(timeDelta, fullUpdate);

	// Solve constraints, repeating until all constraints are satisfied
	if(fullUpdate){
		int iter = 0;

		do{
			// Solve and apply constraints
			for(ContactConstraint& c : contactConstraints_){
				c.solve();
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
			goto constraintLoopExit;

		constraintLoop:;
		} while(iter < PHYS_MAX_CONSTRAINT_ITER);

	constraintLoopExit:;
	}

	// Update all objects
	for(PhysicsObject* obj : dynamicObjects_){
		if(obj->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE || (obj->getPhysicsType() == PhysicsType::DYNAMIC && fullUpdate))
			obj->updatePhysics(timeDelta);
	}
}

vector<ContactManifold>& PhysicsEngine::getContactManifolds(){
	return contactManifolds_;
}
