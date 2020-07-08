#include"physicsEngine.h"

#include"physics/mprCollision.h"
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

	// Generate AABB interval lists
	for(auto i = objects_.begin(); i != objects_.end(); i++){
		if((*i)->getPhysicsType() != PhysicsType::NONE/* && (*i)->getModel()->hitbox != nullptr*/){

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

	// Do broad-phase check with AABBs
	updateAABBCollisionIntervals();

	// Clear contacts and contact constraints from previous update
	contactManifolds_.clear();
	contactConstraints_.clear();


	// For each colliding pair of AABBs
	for(auto i = aabbCollisions_.begin(); i != aabbCollisions_.end(); i++){

		// Ignore pairs that did not collide on all 3 axes
		if(i->second < 3)
			continue;


		Object* object1 = i->first.object1;
		Object* object2 = i->first.object2;


		bool obj1Simple = object1->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;
		bool obj2Simple = object2->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;

		// Only check collisions for simple objects on non-full updates
		if(!fullUpdate && !obj1Simple && !obj2Simple)
			continue;


		// Create collision tester
		SATCollision collisionTest(object1, object2);
		
		// No collision, continue
		if(!collisionTest.testCollision())
			continue;


		// If there is a collision, push back the objects
		ContactManifold m = collisionTest.getContactPoints();

		// Check output valididty
		if(m.contacts.empty())
			continue;

		// Add manifold
		contactManifolds_.push_back(m);

		// If either object uses full rigid body physics, add contact constraints
		if(object1->getPhysicsType() == PhysicsType::DYNAMIC || object2->getPhysicsType() == PhysicsType::DYNAMIC)
			for(Contact& c : contactManifolds_[contactManifolds_.size() - 1].contacts)
				contactConstraints_.push_back(ContactConstraint(c, m.objects));


		// Further collision resolution for objects with simple physics
		if(!obj1Simple && !obj2Simple)
			continue;

		// Resolve collision based on normal of first contact
		const Vec3& normal = m.contacts[0].normal;

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

		/*
		// If a collision occurred, 'binary search' velocities until the objects are as close as possible to not colliding
		int iter = 0;
		float velMult = 0.5;
		float velAdd = 0.25;

		// Adjust velocities and t-update
		Vec3 obj1Velocity = obj1Simple ? ((PhysicsObject*)i->first.object1)->getVelocity() : Vec3();
		Vec3 obj2Velocity = obj2Simple ? ((PhysicsObject*)i->first.object2)->getVelocity() : Vec3();

		//cout << c.normal[0] << "\t" << c.normal[1] << "\t" << c.normal[2] << endl;

		//if(obj1Simple)	i->first.object1->setPosition(i->first.object1->getTPosition() - (c.normal * c.depth));
		//if(obj2Simple)	i->first.object2->setPosition(i->first.object2->getTPosition() + (c.normal * c.depth));

		while(iter < 0){

			// Adjust velocities and t-update
			if(obj1Simple){
				((PhysicsObject*)i->first.object1)->setVelocity(obj1Velocity * velMult);
				((PhysicsObject*)i->first.object1)->tUpdatePhysics(timeDelta);
			}

			if(obj2Simple){
				((PhysicsObject*)i->first.object2)->setVelocity(obj2Velocity * velMult);
				((PhysicsObject*)i->first.object2)->tUpdatePhysics(timeDelta);
			}

			// If still colliding, decrease velocities
			if(SATCollision(i->first.object1, i->first.object2).testCollision())
				velMult -= velAdd;

			// If not colliding, increase velocities
			else
				velMult += velAdd;

			velAdd /= 2;
			iter++;
		}
		*/
	}

	// OBSOLETE RIGID BODY PHYSICS WITH MPA COLLISION
	// NEW CODE USES SAT COLLISION
	/*

	// Keep track of colliding pairs
	vector<ObjectPair> colliding;

	// For each colliding pair of AABBs
	for(auto i = aabbCollisions_.begin(); i != aabbCollisions_.end(); i++){

		// Ignore pairs that did not collide on all 3 axes
		if(i->second < 3)
			continue;
		
		// Create MPR collision tester
		MPRCollision collisionTest(i->first.object1, i->first.object2);

		// Add collision if it occurred
		if(collisionTest.testCollision()){

			// Get contact
			Contact contact = collisionTest.getContact();

			// Check vaildity of contact values
			if(contact.normal.isZero() || contact.normal.isNan())
				goto aabbLoop;

			colliding.push_back(i->first);


			// Check if a contact manifold for the obejct pair already exists
			for(auto j = contactManifolds_.begin(); j != contactManifolds_.end(); j++){

				// Manifold found
				if(i->first == (*j).objects){
					ContactManifold& m = *j;

					// Check validity of existing contacts
					/*
					if(!m.checked){
						for(int k = 0; k < m.contacts.size(); k++){
							if(!checkContactValidity(m.objects, m.contacts[k])){
								m.contacts.erase(m.contacts.begin() + k--);
								continue;
							}

							// Contact is no longer new
							m.contacts[k].isNew = false;
						}

						m.checked = true;
					}
					//*
					m.contacts.clear();

					// Check if the contact is close enough to an existing one
					for(auto k = m.contacts.begin(); k != m.contacts.end(); k++){
						if(
							contact.obj1ContactGlobal.equalsWithinThreshold((*k).obj1ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD) &&
							contact.obj2ContactGlobal.equalsWithinThreshold((*k).obj2ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD)
							){
							// If it is and the old contact is invalid, replace it
							if(!(*k).valid){
								contact.isNew = false;
								*k = contact;
							}

							goto aabbLoop;
						}
					}

					// Add contact to manifold
					m.contacts.push_back(contact);

					// Finished, continue to next AABB collision
					goto aabbLoop;
					/*
					// If the contact is close enough to an existing one, replace the existing one
					for(auto k = m.contacts.begin(); k != m.contacts.end(); k++){
						if(
							contact.obj1ContactGlobal.equalsWithinThreshold((*k).obj1ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD) &&
							contact.obj2ContactGlobal.equalsWithinThreshold((*k).obj2ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD)
							)
						{
							// If the existing contact has not been updated, do so
							if(!(*k).valid){
								*k = contact;
								goto aabbLoop;
							}

							// If it has, add the new contact
							else
								break;
						}
					}

					// Add contact to manifold
					m.contacts.push_back(contact);

					// Limit maximum contacts, erase oldest contact if necessary
					if(m.contacts.size() > PHYS_MAX_CONTACTS)
						m.contacts.erase(m.contacts.begin());

					// Finished, continue to next AABB collision
					goto aabbLoop;
					//*
				}
			}

			// If no manifold was found, create new contact object
			ContactManifold m;
			m.objects = i->first;
			m.contacts = {contact};
			contactManifolds_.push_back(m);
		}

	aabbLoop:;
	}

	// If a contact exists between two objects that are no longer colliding, remove it
	for(int i = 0; i < contactManifolds_.size(); i++){
		for(auto j = colliding.begin(); j != colliding.end(); j++)
			if(*j == contactManifolds_[i].objects)
				goto contactLoop;

		contactManifolds_.erase(contactManifolds_.begin() + i--);
	contactLoop:;
	}

	// Clear contact constraints from previous update
	contactConstraints_.clear();
	
	// Check contact count and add constraints
	for(int i = 0; i < contactManifolds_.size(); i++){
		ContactManifold& m = contactManifolds_[i];

		// Reset checked flag for next update
		m.checked = false;

		// Remove invalid contacts
		for(int k = 0; k < m.contacts.size(); k++){
			if(!m.contacts[k].valid)
				m.contacts.erase(m.contacts.begin() + k--);
			// Reset validity to check again next update
			else
				m.contacts[k].valid = false;
		}

		if(m.contacts.empty()){
			contactManifolds_.erase(contactManifolds_.begin() + i--);
			continue;
		}

		// If there are more than 4 contacts, find which to keep
		if(m.contacts.size() > 4)
			findOptimalContacts(m);

		for(int j = 0; j < m.contacts.size(); j++){

			Contact& c = m.contacts[j];

			// Zero out lambda averages
			c.lambdaAvg		= 0;
			c.lambdaAvgTan1	= 0;
			c.lambdaAvgTan2	= 0;

			// Add contact constraint
			contactConstraints_.push_back(ContactConstraint(c, m.objects));
		}
	}
	*/
}

void PhysicsEngine::updateAABBCollisionIntervals(){

	for(auto obj = objects_.begin(); obj != objects_.end(); obj++){

		// Only need to update semi-dynamic and dynamic objects
		if((*obj)->getPhysicsType() == PhysicsType::NONE || (*obj)->getPhysicsType() == PhysicsType::STATIC)
			continue;

		// If the object has not moved, it does not need to be updated
		if(((PhysicsObject*)*obj)->getVelocity().isZero() && ((PhysicsObject*)*obj)->getAngularVelocity().isZero())
			continue;

		// Get new interval
		vector<float> positions = getCollisionInterval(*obj);

		// Find intervals corresponding to this object
		// NOTE: could be optimized by storing the interval in the object, removing the need to search for it
		for(int i = 0; i < 3; i++){

			// Count intervals updated
			int c = 0;

			for(auto k = aabbCollisionIntervals_[i].begin(); k != aabbCollisionIntervals_[i].end(); k++){

				// If matched, replace position based on start flag
				if((*k).object == *obj){
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

	/*
	for(int i = 0; i < 3; i++){
		vector<CollisionInterval>& intervals = aabbCollisionIntervals_[i];

		for(CollisionInterval& i : intervals)
			cout << i.position << "\t";

		cout << endl;
	}

	cout << endl;
	*/
}

vector<float> PhysicsEngine::getCollisionInterval(Object* obj){

	//Vec3 position = obj->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)obj)->getTPosition() : obj->getPosition();
	//Vec3 scale = obj->getScale();
	//Quaternion rotation = obj->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)obj)->getTRotation() : obj->getRotation();
	//Matrix transform = Matrix(3, 3, true);

	float x1, x2, y1, y2, z1, z2;
	x1 = y1 = z1 = std::numeric_limits<float>::max();
	x2 = y2 = z2 = -x1;

	const Hitbox& hitbox = obj->getTransformedHitboxSAT();

	for(const Vec3& v : hitbox.vertices){
		x1 = min(x1, v[0]);	x2 = max(x2, v[0]);
		y1 = min(y1, v[1]);	y2 = max(y2, v[1]);
		z1 = min(z1, v[2]);	z2 = max(z2, v[2]);
	}

	/*
	switch(obj->getHitboxType()){
	case HitboxType::MESH:
		transform.rotate(rotation);

		// Use pre-defined half-cube vertices for hitbox
		for(auto i = cubeVerts_.begin(); i != cubeVerts_.end(); i++){
			Vec3 v1 = *i;

			// Scale
			v1.setX(v1[0] * scale[0]);
			v1.setY(v1[1] * scale[1]);
			v1.setZ(v1[2] * scale[2]);

			// Rotate
			v1 = transform * v1;

			// Use this vertex and its mirror around the cube position
			// This allows the cube hitbox mesh to be half the size
			Vec3 v2 = position - v1;
			v1 += position;

			x1 = min(min(x1, v1[0]), v2[0]);	x2 = max(max(x2, v1[0]), v2[0]);
			y1 = min(min(y1, v1[1]), v2[1]);	y2 = max(max(y2, v1[1]), v2[1]);
			z1 = min(min(z1, v1[2]), v2[2]);	z2 = max(max(z2, v1[2]), v2[2]);
		}

		break;

	case HitboxType::SPHERE:
		x1 = position[0] - scale[0];	x2 = position[0] + scale[0];
		y1 = position[1] - scale[1];	y2 = position[1] + scale[1];
		z1 = position[2] - scale[2];	z2 = position[2] + scale[2];
		break;


		// TEMPORARY
	case HitboxType::PREDEFINED:{
		transform.rotate(rotation);

		// Get unique vertices
		vector<float>& objVerts = obj->getModel()->vertices;
		vector<Vec3> verts;

		for(auto j = 0; j < objVerts.size() / 3; j++){

			Vec3 v = Vec3(
				objVerts[j * 3],
				objVerts[j * 3 + 1],
				objVerts[j * 3 + 2]
			);

			if(std::find(verts.begin(), verts.end(), v) == verts.end())
				verts.push_back(v);
		}


		for(auto j = verts.begin(); j != verts.end(); j++){

			// Scale
			(*j).setX((*j)[0] * scale[0]);
			(*j).setY((*j)[1] * scale[1]);
			(*j).setZ((*j)[2] * scale[2]);

			// Rotate
			*j = transform * *j;

			// Translate
			*j += position;

			x1 = min(x1, (*j)[0]);	x2 = max(x2, (*j)[0]);
			y1 = min(y1, (*j)[1]);	y2 = max(y2, (*j)[1]);
			z1 = min(z1, (*j)[2]);	z2 = max(z2, (*j)[2]);
		}
		break;
	}

	}
	*/

	return vector<float>{x1, x2, y1, y2, z1, z2};
}

bool PhysicsEngine::checkContactValidity(ObjectPair& objects, Contact& c){
	
	Object* obj1 = objects.object1;
	Object* obj2 = objects.object2;
	bool obj1Dynamic = obj1->getPhysicsType() == PhysicsType::DYNAMIC;
	bool obj2Dynamic = obj2->getPhysicsType() == PhysicsType::DYNAMIC;
	Vec3 obj1Pos = obj1Dynamic ? ((PhysicsObject*)obj1)->getTPosition() : obj1->getPosition();
	Vec3 obj2Pos = obj2Dynamic ? ((PhysicsObject*)obj2)->getTPosition() : obj2->getPosition();

	Matrix obj1Rotation = Matrix(3, 3, true).rotate(
		obj1Dynamic ? ((PhysicsObject*)obj1)->getTRotation() : obj1->getRotation()
	);
	Matrix obj2Rotation = Matrix(3, 3, true).rotate(
		obj2Dynamic ? ((PhysicsObject*)obj2)->getTRotation() : obj2->getRotation()
	);

	// Get updated global contact points
	Vec3 obj1ContactGlobalU = c.obj1ContactLocal;
	Vec3 obj2ContactGlobalU = c.obj2ContactLocal;

	// Rotate
	obj1ContactGlobalU = obj1Rotation * obj1ContactGlobalU;
	obj2ContactGlobalU = obj2Rotation * obj2ContactGlobalU;

	// Translate
	obj1ContactGlobalU += obj1Pos;
	obj2ContactGlobalU += obj2Pos;

	// If valid, set as persistent
	c.valid =
		c.normal * (obj2ContactGlobalU - obj1ContactGlobalU) >= 0;// &&
	//	obj1ContactGlobalU.equalsWithinThreshold(c.obj1ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD) &&
	//	obj2ContactGlobalU.equalsWithinThreshold(c.obj2ContactGlobal, PHYS_CONTACT_UPDATE_THRESHOLD);

	//c.valid = true;

	// Update contact points and vectors
	/*
	if(c.valid){
		Vec3 obj1Diff = obj1ContactGlobalU - c.obj1ContactGlobal;
		Vec3 obj2Diff = obj2ContactGlobalU - c.obj2ContactGlobal;
		c.obj1ContactGlobal += obj1Diff + obj2Diff - obj2Diff.compOn(c.normal);
		c.obj2ContactGlobal += obj2Diff + obj1Diff - obj1Diff.compOn(c.normal);
		c.obj1ContactVector = c.obj1ContactGlobal - obj1Pos;
		c.obj2ContactVector = c.obj2ContactGlobal - obj2Pos;
	}
	*/
	return c.valid;
}

void PhysicsEngine::findOptimalContacts(ContactManifold& m){

	// First contact is the one with deepest penetration
	Contact* first = &m.contacts[0];
	float dist = 0;
	for(auto i = m.contacts.begin(); i != m.contacts.end(); i++){
		float curDist = (*i).depth;

		if(curDist > dist){
			first = &*i;
			dist = curDist;
		}
	}

	Vec3 a = first->obj1ContactVector;


	// Second contact is the one furthest from the first
	Contact* second = &m.contacts[1];
	dist = 0;
	for(auto i = m.contacts.begin(); i != m.contacts.end(); i++){
		float curDist = (a - (*i).obj1ContactVector).magnitude2();

		if(curDist > dist){
			second = &*i;
			dist = curDist;
		}
	}

	Vec3 b = second->obj1ContactVector;


	// Third contact
	Contact* third = &m.contacts[2];
	dist = 0;
	for(auto i = m.contacts.begin(); i != m.contacts.end(); i++){
		float curDist =	(a - (*i).obj1ContactVector).magnitude2() +
						(b - (*i).obj1ContactVector).magnitude2();
		/*
		float curDist = 0;

		// Find distance between line segment of first two contacts and point of current contact
		Vec3 v = second->obj1ContactVector - first->obj1ContactVector;
		Vec3 w = (*i).obj1ContactVector - first->obj1ContactVector;

		float c1 = v * w;

		if(c1 <= 0)
			curDist = w.magnitude2();
		else{
			float c2 = v.magnitude2();

			if(c2 <= c1)
				curDist = ((*i).obj1ContactVector - second->obj1ContactVector).magnitude2();
			else
				curDist = (
					(*i).obj1ContactVector - 
					(first->obj1ContactVector + ((c1 / c2) * v))
				).magnitude2();
		}
		*/
		if(curDist > dist){
			third = &*i;
			dist = curDist;
		}
	}

	Vec3 c = third->obj1ContactVector;


	// Fourth contact
	Contact* fourth = &m.contacts[3];
	dist = 0;
	for(auto i = m.contacts.begin(); i != m.contacts.end(); i++){
		float curDist =	(a - (*i).obj1ContactVector).magnitude2() +
						(b - (*i).obj1ContactVector).magnitude2() +
						(c - (*i).obj1ContactVector).magnitude2();

		if(curDist > dist){
			fourth = &*i;
			dist = curDist;
		}
	}

	Vec3 d = fourth->obj1ContactVector;


	// Add contacts
	m.contacts.clear();
	m.contacts.push_back(*first);
	m.contacts.push_back(*second);
	m.contacts.push_back(*third);

	// Add fourth contact if it is not within the triangle formed by the first 3
	auto l_sameSide = [](Vec3 p1, Vec3 p2, Vec3 a, Vec3 b) -> bool{
		Vec3 ba = b - a;
		return crossProduct(ba, p1 - a) * crossProduct(ba, p2 - a) >= 0;
	};

	if(!l_sameSide(d, a, b, c) || !l_sameSide(d, b, a, c) || !l_sameSide(d, c, a, b))
		m.contacts.push_back(*fourth);
}
