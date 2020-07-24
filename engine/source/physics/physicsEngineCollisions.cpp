#include"physicsEngine.h"

#include"physics/satCollision.h"
#include"physics/physDefine.h"
#include"math/mathFunc.h"
#include"core/error.h"
#include<algorithm>
#include<limits>

using ntw::crossProduct;
using std::min;
using std::max;


void PhysicsEngine::initAABBCollisions(){

	// Clear lists
	for(vector<CollisionInterval>& list : aabbCollisionIntervals_)
		list.clear();

	// Generate AABB interval lists
	for(auto i = objects_.begin(); i != objects_.end(); i++){
		if((*i)->getPhysicsType() != PhysicsType::NONE && (*i)->getHitboxType() != HitboxType::NONE){

			// Add intervals
			vector<float> positions = getCollisionInterval(*i);

			for(auto j = 0; j < positions.size(); j++)
				aabbCollisionIntervals_[j / 2].push_back(CollisionInterval{positions[j], j % 2 == 0, *i});
		}
	}

	// Sort interval lists
	std::sort(aabbCollisionIntervals_[0].begin(), aabbCollisionIntervals_[0].end());
	std::sort(aabbCollisionIntervals_[1].begin(), aabbCollisionIntervals_[1].end());
	std::sort(aabbCollisionIntervals_[2].begin(), aabbCollisionIntervals_[2].end());


	// Find AABB collisions
	aabbCollisions_.clear();

	// Check for each axis
	for(int i = 0; i < 3; i++){
		vector<CollisionInterval*> active;

		for(auto j = aabbCollisionIntervals_[i].begin(); j != aabbCollisionIntervals_[i].end(); j++){

			// If this is the start of an interval, mark it active
			if((*j).start){
				active.push_back(&*j);

				// If it overlaps with another active interval, the two objects are colliding on this axis
				for(auto k = active.begin(); k != active.end() - 1; k++){

					// Only need to process collisions with dynamic objects
					if((*j).object->getPhysicsType() != PhysicsType::DYNAMIC && (*j).object->getPhysicsType() != PhysicsType::DYNAMIC_SIMPLE &&
						(*k)->object->getPhysicsType() != PhysicsType::DYNAMIC && (*k)->object->getPhysicsType() != PhysicsType::DYNAMIC_SIMPLE)
						continue;

					ObjectPair collision = ObjectPair{(*j).object, (*k)->object};

					// If the collision is already present, increment its count
					auto col = aabbCollisions_.find(collision);

					if(col != aabbCollisions_.end())
						col->second++;

					// Otherwise, add it to the list
					else
						aabbCollisions_.emplace(collision, 1);
				}
			}

			// If this is the end of an interval, remove it from active
			else{
				for(auto k = active.begin(); k != active.end(); k++){
					// Find matching start of interval
					if((*k)->object == (*j).object){
						active.erase(k);
						break;
					}
				}
			}
		}
	}
}

#include<iostream>
using std::cout;
using std::endl;

void PhysicsEngine::checkCollisions(float timeDelta, bool fullUpdate){

	// Broadphase AABB check
	aabbTree_.update();
	const vector<AABBPair>& overlappingAABBs = aabbTree_.getOverlapping();

	if(!overlappingAABBs.empty())
		cout << overlappingAABBs.size() << endl;

	return;

	//updateAABBCollisionIntervals();

	// Clear contacts and contact constraints from previous update
	contactManifolds_.clear();
	contactConstraints_.clear();


	// Find colliding AABBs
	vector<ObjectPair> aabbColliding;

	for(auto i = aabbCollisions_.begin(); i != aabbCollisions_.end(); i++){

		// Ignore pairs that did not collide on all 3 axes
		if(i->second < 3)
			continue;

		bool obj1Simple = i->first.object1->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;
		bool obj2Simple = i->first.object2->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;

		// Only check collisions for simple objects on non-full updates
		if(!fullUpdate && !obj1Simple && !obj2Simple)
			continue;

		// Add collision to list
		//aabbColliding.push_back(i->first);

		// Check for and resolve collision
		resolveCollision(i->first, timeDelta);
	}

	// Start resolving collisions from static objects
	/*
	Object* active = nullptr;
	bool finished = false;

	while(!finished){
		for(auto i = aabbColliding.begin(); i != aabbColliding.end(); ){

			// No active object, look for a static object
			if(active == nullptr){
				if((*i).object1->getPhysicsType() == PhysicsType::STATIC)	active = (*i).object1;
				if((*i).object2->getPhysicsType() == PhysicsType::STATIC)	active = (*i).object2;
			}

			// Resolve 
		}
	}
	*/
}

void PhysicsEngine::resolveCollision(const ObjectPair& objects, float timeDelta){

	Object* object1 = objects.object1;
	Object* object2 = objects.object2;

	bool obj1Simple = object1->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;
	bool obj2Simple = object2->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;

	// Create collision tester
	SATCollision collisionTest(object1, object2);

	// No collision, return
	if(!collisionTest.testCollision())
		return;


	// If there is a collision, push back the objects
	ContactManifold m = collisionTest.getContactPoints();

	// Check output valididty
	if(m.contacts.empty())
		return;


	// Add manifold
	contactManifolds_.push_back(m);


	// Use normal of first contact as normal for entire manifold
	const Vec3& normal = m.contacts[0].normal;

	// Add contact info to objects
	object1->addContact({object2, normal});
	object2->addContact({object1, normal});


	// If either object uses full rigid body physics, add contact constraints
	if(object1->getPhysicsType() == PhysicsType::DYNAMIC || object2->getPhysicsType() == PhysicsType::DYNAMIC)
		for(Contact& c : contactManifolds_[contactManifolds_.size() - 1].contacts)
			contactConstraints_.push_back(ContactConstraint(c, m.objects));


	// Further collision resolution for objects with simple physics
	if(!obj1Simple && !obj2Simple)
		return;


	float distance = m.maxDistance;
	float distMult = 0;

	// Distance is maximum penetration depth
	for(const Contact& c : m.contacts)
		if(c.depth > distance)
			distance = c.depth;

	// Amount to push back based on mass
	if(object1->getPhysicsType() != PhysicsType::NONE && object1->getPhysicsType() != PhysicsType::STATIC){
		if(object2->getPhysicsType() != PhysicsType::NONE && object2->getPhysicsType() != PhysicsType::STATIC){
			float obj1Mass = ((PhysicsObject*)object1)->getMass();
			float obj2Mass = ((PhysicsObject*)object2)->getMass();
			distMult = obj2Mass / (obj1Mass + obj2Mass);
		}
		else
			distMult = 1;
	}

	bool obj1Phys = obj1Simple || object1->getPhysicsType() == PhysicsType::DYNAMIC;
	bool obj2Phys = obj2Simple || object2->getPhysicsType() == PhysicsType::DYNAMIC;

	// Zero object velocity and push outwards along penetration normal
	if(obj1Phys && distMult > 0){
		float dist = distance * distMult;
		const Vec3& vel = ((PhysicsObject*)object1)->getVelocity();

		// Get projected velocity
		Vec3 proj = vel.clampedProjOn(-normal);

		// Set on ground if angle is > 45 degrees
		if(normal * Vec3(0, 0, 1) >= 0.707){
			((PhysicsObject*)object1)->setOnGround(true);

			// Remove horizontal components from velocity adjustment
			proj[0] = 0;
			proj[1] = 0;
		}

		// Zero velocity
		((PhysicsObject*)object1)->setVelocity(vel - proj);

		// Push back based on projected velocity and penetration distance
		object1->setPosition(object1->getPosition() - (dist * -normal) + (proj * timeDelta));
	}
	if(obj2Phys && (1 - distMult) > 0){
		float dist = distance * (1 - distMult);
		const Vec3& vel = ((PhysicsObject*)object2)->getVelocity();

		// Get projected velocity
		Vec3 proj = vel.clampedProjOn(normal);

		// Set on ground if angle is > 45 degrees
		if(-normal * Vec3(0, 0, 1) >= 0.707){
			((PhysicsObject*)object2)->setOnGround(true);

			// Remove horizontal components from velocity adjustment
			proj[0] = 0;
			proj[1] = 0;
		}

		// Zero velocity
		((PhysicsObject*)object2)->setVelocity(vel - proj);

		// Push back based on projected velocity and penetration distance
		object2->setPosition(object2->getPosition() - (dist * normal) + (proj * timeDelta));
	}
}

void PhysicsEngine::updateAABBCollisionIntervals(){

	// Only need to update semi-dynamic and dynamic objects
	for(PhysicsObject* object : dynamicObjects_){

		// If the object has not moved, it does not need to be updated
		if(object->getVelocity().isZero() && object->getAngularVelocity().isZero())
			continue;

		// Get new interval
		vector<float> positions = getCollisionInterval(object);

		// Find intervals corresponding to this object
		// NOTE: could be optimized by storing the interval in the object, removing the need to search for it
		for(int i = 0; i < 3; i++){

			// Count intervals updated
			int c = 0;

			for(auto k = aabbCollisionIntervals_[i].begin(); k != aabbCollisionIntervals_[i].end(); k++){

				// If matched, replace position based on start flag
				if((*k).object == object){
					(*k).position = (*k).start ? positions[i * 2] : positions[i * 2 + 1];
					c++;

					// Once both intervals have been added, exit this loop
					if(c == 2)
						goto intervalLoop;
				}
			}
		intervalLoop:;
		}
	}

	// Re-sort intervals and update collisions
	for(int i = 0; i < 3; i++){
		vector<CollisionInterval>& intervals = aabbCollisionIntervals_[i];

		// Do insertion sort
		for(int j = 1; j < intervals.size(); j++){

			int k = j;
			while(k > 0 && intervals[k] < intervals[k - 1]){

				// Make sure first interval is the start if they are equal and from the same object
				// The less than operation will be true if the intervals are equal but in the wrong order
				if(intervals[k] == intervals[k - 1]){
					intervals[k - 1].start = true;
					intervals[k].start = false;
					break;
				}

				// Perform swap
				CollisionInterval tmp = intervals[k];
				intervals[k] = intervals[k - 1];
				intervals[k - 1] = tmp;

				// If a swap was performed, update collisions list
				CollisionInterval& first = intervals[k - 1];
				CollisionInterval& second = intervals[k];

				// If both are the same type, no change is performed
				// Only need to process collisions with dynamic objects
				if((first.start ^ second.start) &&
					(first.object->getPhysicsType() == PhysicsType::DYNAMIC || first.object->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE ||
					second.object->getPhysicsType() == PhysicsType::DYNAMIC	|| second.object->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE)){

					ObjectPair collision = ObjectPair{first.object, second.object};

					// Find collision if it is already present
					auto col = aabbCollisions_.find(collision);

					// If the lesser is now the start of an interval, the two objects are now colliding
					if(first.start){
						// If the collision is already present, increment its count
						if(col != aabbCollisions_.end())
							col->second++;

						// Otherwise, add it to the list
						else
							aabbCollisions_.emplace(collision, 1);
					}

					// If the lesser is now the end of an interval, the two objects are no longer colliding
					else if(col != aabbCollisions_.end())
						col->second--;
				}

				k--;
			}
		}
	}
}

vector<float> PhysicsEngine::getCollisionInterval(Object* obj){

	// Min/max coordinate values
	float x1, x2, y1, y2, z1, z2;
	x1 = y1 = z1 = std::numeric_limits<float>::max();
	x2 = y2 = z2 = -x1;

	// Cache object transformed hitbox
	obj->cacheTransformedHitbox();
	const vector<Collider>& colliders = obj->getColliders();

	// Get bounding coordinates
	for(const Collider& c : colliders){
		for(const Vec3& v : c.hitboxTransformed.vertices){
			x1 = min(x1, v[0]);	x2 = max(x2, v[0]);
			y1 = min(y1, v[1]);	y2 = max(y2, v[1]);
			z1 = min(z1, v[2]);	z2 = max(z2, v[2]);
		}
	}

	return vector<float>{x1, x2, y1, y2, z1, z2};
}
