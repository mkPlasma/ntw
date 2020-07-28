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

	// Make portal collisions out of date
	for(auto& i : portalCollisions_)
		i.second.updated = false;


	// Broadphase AABB check
	aabbTree_.update();
	const vector<AABBPair>& overlappingAABBs = aabbTree_.getOverlapping();

	// Check for and resolve collisions
	for(const AABBPair& pair : overlappingAABBs)
		resolveCollision(pair);


	// Remove out-of-date portal collisions
	for(auto i = portalCollisions_.begin(); i != portalCollisions_.end();){
		if(!i->second.updated)
			i = portalCollisions_.erase(i);
		else
			i++;
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

void PhysicsEngine::resolveCollision(const AABBPair& pair){

	// Get colliders
	const Collider* collider1 = pair.aabb1.collider;
	const Collider* collider2 = pair.aabb2.collider;

	// Get objects
	Object* object1 = collider1->parent;
	Object* object2 = collider2->parent;


	// Portal collisions
	if(!object1 || !object2){
		resolvePortalCollision(!object1 ? object2 : object1, !object1 ? collider1->portal : collider2->portal);
		return;
	}


	// Create collision tester
	SATCollision collisionTest(collider1, collider2);

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
