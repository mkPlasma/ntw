#include"physicsEngine.h"

#include"physics/satCollision.h"
#include"physics/physDefine.h"
#include"objects/portal.h"
#include"objects/player.h"
#include"math/mathFunc.h"
#include"core/error.h"
#include<algorithm>
#include<limits>

using ntw::crossProduct;
using std::min;
using std::max;


void PhysicsEngine::checkCollisions(){

	// Clear contacts and contact constraints from previous update
	contactManifolds_.clear();
	contactConstraints_.clear();


	// Make SAT collisions and portal collisions out of date
	for(auto& i : satCollisions_)
		i.second.updated = false;

	for(auto& i : portalCollisions_)
		i.second.updated = false;


	// Broadphase AABB check
	aabbTree_.update();
	const vector<AABBPair>& overlappingAABBs = aabbTree_.getOverlapping();


	// Check for and resolve collisions, first with portals then with objects
	for(int i = 0; i < 2; i++){
		for(const AABBPair& pair : overlappingAABBs){

			// Get objects
			Object* object1 = pair.aabb1.collider->parent;
			Object* object2 = pair.aabb2.collider->parent;

			// Portal collision
			if(!object1 || !object2){
				if(i == 0)
					resolvePortalCollision(!object1 ? object2 : object1, !object1 ? pair.aabb1.collider->portal : pair.aabb2.collider->portal);
			}
			else if(i == 1)
				resolveCollision(pair);
		}
	}


	// Remove out-of-date SAT collisions and portal collisions
	for(auto i = satCollisions_.begin(); i != satCollisions_.end();){
		if(!i->second.updated)
			i = satCollisions_.erase(i);
		else
			i++;
	}

	for(auto i = portalCollisions_.begin(); i != portalCollisions_.end();){
		if(!i->second.updated)
			i = portalCollisions_.erase(i);
		else
			i++;
	}
}

void PhysicsEngine::resolveCollision(const AABBPair& pair){

	// Get colliders
	const Collider* collider1 = pair.aabb1.collider;
	const Collider* collider2 = pair.aabb2.collider;

	// Get objects
	Object* object1 = collider1->parent;
	Object* object2 = collider2->parent;
	ObjectPair objectPair = {object1, object2};


	// Create collision tester
	SATCollision collisionTest(collider1, collider2);


	// Check if there is a cached collision result
	auto i = satCollisions_.find(objectPair);

	ContactManifold m;

	if(i != satCollisions_.end()){
		
		SATCollisionInfo& info = i->second;

		// Check if cached result is valid
		if(info.collided){
			// Objects collided, try to generate the same contact points
			collisionTest.setContactInfo(info.contactInfo);
			m = collisionTest.getContactPoints();

			// No contacts generated, redo test
			if(m.contacts.empty()){
				if(!collisionTest.testCollision()){
					// No collision, invalidate contact info and return
					info.updated = false;
					return;
				}

				// Objects are colliding
				m = collisionTest.getContactPoints();
			}
		}
		else{
			// Objects did not collide, do test with separating axis
			if(!collisionTest.testCollision(info.separatingAxis))
				return;

			// Collision found, invalidate separating axis
			m = collisionTest.getContactPoints();
			info.updated = false;
		}
	}

	// No cached result, test collision and cache
	else{
		SATCollisionInfo info;
		info.collided = collisionTest.testCollision();

		if(!info.collided){
			// No collision, cache separating axis and return
			info.separatingAxis = collisionTest.getSeparatingAxis();
			satCollisions_.emplace(objectPair, info);

			return;
		}

		// Collision found, cache contact info
		m = collisionTest.getContactPoints();

		info.contactInfo = collisionTest.getContactInfo();
		satCollisions_.emplace(objectPair, info);
	}

	

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
	if(object1->getPhysicsType() == PhysicsType::RIGID_BODY || object2->getPhysicsType() == PhysicsType::RIGID_BODY)
		for(Contact& c : contactManifolds_[contactManifolds_.size() - 1].contacts)
			contactConstraints_.push_back(ContactConstraint(c, m.objects));


	// Further collision resolution for objects with simple physics
	bool obj1Simple = object1->getPhysicsType() == PhysicsType::SIMPLE;
	bool obj2Simple = object2->getPhysicsType() == PhysicsType::SIMPLE;

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

	bool obj1Phys = obj1Simple || object1->getPhysicsType() == PhysicsType::RIGID_BODY;
	bool obj2Phys = obj2Simple || object2->getPhysicsType() == PhysicsType::RIGID_BODY;

	// Zero object velocity and push outwards along penetration normal
	if(obj1Phys && distMult > 0){
		float dist = distance * distMult;
		const Vec3& vel = ((PhysicsObject*)object1)->getVelocity();

		// Get projected velocity
		Vec3 proj = vel.clampedProjOn(-normal);

		// Set on ground if angle is > 45 degrees
		if(normal * -((PhysicsObject*)object1)->getGravityDirection() >= 0.707)
			((PhysicsObject*)object1)->setOnGround(true);

		// Zero velocity
		((PhysicsObject*)object1)->setVelocity(vel - proj);

		// Push back based on projected velocity and penetration distance
		object1->setPosition(object1->getPosition() - (dist * -normal) + (proj * NTW_PHYS_TIME_DELTA));
	}
	if(obj2Phys && (1 - distMult) > 0){
		float dist = distance * (1 - distMult);
		const Vec3& vel = ((PhysicsObject*)object2)->getVelocity();

		// Get projected velocity
		Vec3 proj = vel.clampedProjOn(normal);

		// Set on ground if angle is > 45 degrees
		if(-normal * -((PhysicsObject*)object2)->getGravityDirection() >= 0.707)
			((PhysicsObject*)object2)->setOnGround(true);

		// Zero velocity
		((PhysicsObject*)object2)->setVelocity(vel - proj);

		// Push back based on projected velocity and penetration distance
		object2->setPosition(object2->getPosition() - (dist * normal) + (proj * NTW_PHYS_TIME_DELTA));
	}
}

void PhysicsEngine::resolvePortalCollision(Object* object, Portal* portal){

	// Ignore if object is not within portal clip planes
	// In other words, object collides with the portal AABB but is beside it
	if(!portal->isPointWithinClipPlanes(object->getPosition()))
		return;

	ObjectPortalPair pair = {object, portal};

	// Get side of portal object is on
	bool objectInFront = portal->isPointInFront(object->getPosition());

	// Check if pair exists
	auto i = portalCollisions_.find(pair);

	if(i != portalCollisions_.end()){

		PortalCollisionInfo& info = i->second;

		// Collision has been updated
		info.updated = true;

		// Teleport object if it is now on the other side of the portal
		if(info.objectInFront ^ objectInFront){

			// Teleport
			object->setPosition(portal->getTransformedVector(object->getPosition()));

			// Set rotation for player
			if(object->isPlayer()){
				((Player*)object)->addPortalRotation(portal->getRotationMatrix());
				((Player*)object)->updateEyePosition();
			}

			// Set rotation for objects
			else
				object->setRotation(Quaternion(portal->getRotationMatrix()) * object->getRotation());

			// Set velocity, angular velocity, and gravity direction
			PhysicsObject* pObj = (PhysicsObject*)object;
			pObj->setVelocity(portal->getRotatedVector(pObj->getVelocity()));
			pObj->setAngularVelocity(portal->getRotatedVector(pObj->getAngularVelocity()));
			pObj->setGravityDirection(portal->getRotatedVector(pObj->getGravityDirection()));


			// Remove this collision and add another with the pair portal
			portalCollisions_.erase(i);

			ObjectPortalPair newPair = {object, portal->getPairedPortal()};
			PortalCollisionInfo newInfo = {newPair, !objectInFront, true};
			portalCollisions_.emplace(newPair, newInfo);
		}

		return;
	}
	
	// Pair not found, add it
	PortalCollisionInfo info = {pair, objectInFront, true, object->isPlayer()};
	portalCollisions_.emplace(pair, info);
}
