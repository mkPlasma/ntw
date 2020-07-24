#include"physicsEngine.h"

#include"physDefine.h"
#include"core/error.h"
#include"core/world.h"
#include"physics/satCollision.h"
#include"objects/modelFunc.h"
#include<algorithm>

using std::max;


PhysicsEngine::PhysicsEngine(World& world, vector<Object*>& objects, vector<PhysicsObject*>& physicsObjects)
	: world_(world), objects_(objects), dynamicObjects_(physicsObjects), initialized_(false) {

	// AABB collision interval lists
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
	aabbCollisionIntervals_.push_back(vector<CollisionInterval>());
}

void PhysicsEngine::init(){

	// Initialize physics objects and add colliders to AABB tree
	for(Object* obj : objects_)
		initObject(obj);

	initAABBCollisions();

	initialized_ = true;
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

				// Temp update objects
				if(c.getObjects().object1->getPhysicsType() == PhysicsType::DYNAMIC)
					((PhysicsObject*)c.getObjects().object1)->tUpdatePhysics(timeDelta);

				if(c.getObjects().object2->getPhysicsType() == PhysicsType::DYNAMIC)
					((PhysicsObject*)c.getObjects().object2)->tUpdatePhysics(timeDelta);
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
		} while(iter < NTW_PHYS_MAX_CONSTRAINT_ITER);

	constraintLoopExit:;
	}

	// Update all objects
	for(PhysicsObject* obj : dynamicObjects_){
		if(obj->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE || (obj->getPhysicsType() == PhysicsType::DYNAMIC && fullUpdate))
			obj->updatePhysics(timeDelta);
	}
}


vector<Object*> PhysicsEngine::castRay(const Vec3& position, const Vec3& direction, float maxDistance){

	Vec3 endPos = direction.unitVector() * maxDistance;

	// Create ray model
	Model* model = new Model();
	model->vertices = {0, 0, 0, endPos[0], endPos[1], endPos[2]};
	model->numVertices = 2;

	// Ray hitbox
	model->hitboxSAT.vertices.push_back(Vec3(0, 0, 0));
	model->hitboxSAT.vertices.push_back(endPos);
	model->hitboxSAT.edges.push_back({0, 1, -1, -1});


	// Create ray object
	PhysicsObject* ray = new PhysicsObject(world_, model, nullptr, HitboxType::MESH, 0);
	ray->setRenderType(RenderType::NONE);
	ray->setPosition(position);

	// List of colliding objects
	vector<Object*> colliding;

	// Add object and test collisions
	world_.addObject(ray);

	for(auto i = aabbCollisions_.begin(); i != aabbCollisions_.end(); i++){

		if(i->second < 3)
			continue;

		Object* object1 = i->first.object1;
		Object* object2 = i->first.object2;

		if(object1 != ray && object2 != ray)
			continue;

		// Check collision and add object to list if colliding
		SATCollision collisionTest(object1, object2);

		if(collisionTest.testCollision()){
			if(object1 == ray)	colliding.push_back(object2);
			else				colliding.push_back(object1);
		}
	}

	// Delete ray
	world_.removeObject(ray);
	delete model;
	delete ray;
	
	return colliding;
}

void PhysicsEngine::initObject(Object* object){

	// Initialize physics
	if(object->getPhysicsType() == PhysicsType::DYNAMIC || object->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE)
		((PhysicsObject*)object)->initPhysics();


	// Add colliders to AABB tree
	const vector<Collider>& colliders = object->getColliders();

	for(const Collider& c : colliders)
		aabbTree_.add(&c);


	if(!initialized_ || object->getHitboxType() == HitboxType::NONE)
		return;


	// Add to AABB lists if necessary
	vector<float> positions = getCollisionInterval(object);

	for(auto i = 0; i < positions.size(); i++){
		vector<CollisionInterval>& list = aabbCollisionIntervals_[i / 2];
		float pos = positions[i];
		CollisionInterval ci = {positions[i], i % 2 == 0, object};

		// Insert at correct position in list
		if(list.empty() || list[0].position > pos){
			list.insert(list.begin(), ci);
			continue;
		}

		for(auto j = list.begin(); j != list.end() - 1; j++){
			if(pos > (*j).position && (j + 1) == list.end() || pos < (*(j + 1)).position){
				list.insert(j + 1, ci);
				break;
			}
		}
	}

	// TEMPORARY
	// re-initialize aabb lists
	initAABBCollisions();
}

void PhysicsEngine::removeObject(Object* object){

	if(!initialized_ || object->getHitboxType() == HitboxType::NONE)
		return;

	// Remove from AABB lists
	for(vector<CollisionInterval>& list : aabbCollisionIntervals_){
		int removed = 0;

		for(auto i = list.begin(); i != list.end(); i++){
			if((*i).object == object){
				i = list.erase(i);
				removed++;

				if(removed >= 2)
					break;
			}
		}
	}
	
	for(auto i = aabbCollisions_.begin(); i != aabbCollisions_.end();){
		if(i->first.object1 == object || i->first.object2 == object)
			i = aabbCollisions_.erase(i);
		else
			i++;
	}
}


bool PhysicsEngine::isInitialized(){
	return initialized_;
}

vector<ContactManifold>& PhysicsEngine::getContactManifolds(){
	return contactManifolds_;
}

AABBTree::Node* PhysicsEngine::getRoot(){
	return aabbTree_.getRoot();
}
