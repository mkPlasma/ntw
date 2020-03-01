#include"mprCollision.h"

#include"core/error.h"
#include"math/matrix.h"
#include"physDefine.h"
#include<limits>

using ntw::crossProduct;


MPRCollision::MPRCollision(Object* object1, Object* object2) : object1_(object1), object2_(object2) {
	init();
}

void MPRCollision::init(){

	bool obj1Dynamic = object1_->getPhysicsType() == PHYS_DYNAMIC;
	bool obj2Dynamic = object2_->getPhysicsType() == PHYS_DYNAMIC;

	obj1Pos_		= obj1Dynamic ? ((PhysicsObject*)object1_)->getTPosition() : object1_->getPosition();
	obj2Pos_		= obj2Dynamic ? ((PhysicsObject*)object2_)->getTPosition() : object2_->getPosition();
	obj1Rotation_	= Matrix(3, 3, true).rotate(obj1Dynamic ? ((PhysicsObject*)object1_)->getTRotation() : object2_->getRotation());
	obj2Rotation_	= Matrix(3, 3, true).rotate(obj2Dynamic ? ((PhysicsObject*)object2_)->getTRotation() : object2_->getRotation());

	// Cache transformed hitbox vertices
	for(int i = 0; i < 2; i++){
		Object*& obj = i == 0 ? object1_ : object2_;
		vector<Vec3>& verts = i == 0 ? object1Verts_ : object2Verts_;

		Vec3 position = i == 0 ? obj1Pos_ : obj2Pos_;
		Vec3 scale = obj->getScale();
		Matrix rotation = i == 0 ? obj1Rotation_ : obj2Rotation_;

		switch(obj->getHitboxType()){
		case HITBOX_CUBE:
			// Use pre-defined cube vertices
			for(auto j = cubeVerts_.begin(); j != cubeVerts_.end(); j++){

				Vec3 v = *j;

				// Scale
				v.setX(v[0] * scale[0]);
				v.setY(v[1] * scale[1]);
				v.setZ(v[2] * scale[2]);

				// Rotate
				v = rotation * v;

				// Translate
				v += position;

				verts.push_back(v);
			}
			break;


			// TEMPORARY
		case HITBOX_MESH:{

			// Get unique vertices
			vector<float>& objVerts = obj->getModel()->vertices;

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
				*j = rotation * *j;

				// Translate
				*j += position;
			}
			break;
		}
		}
	}
}

bool MPRCollision::testCollision(){
	
	// First point in center of Minkowski difference
	vertices_.push_back({obj2Pos_ - obj1Pos_, obj1Pos_, obj2Pos_});
	if(vertices_[0].v == zero_)	vertices_[0].v = Vec3(0, 0, 0.0001f);


	// Second point is support in direction of origin
	direction_ = -vertices_[0].v;
	if(addSupportPoint())
		return false;


	// Check if the vectors are parallel
	direction_ = crossProduct(vertices_[0].v, vertices_[1].v);
	if(direction_ == zero_)
		return true;


	// Third point perpendicular to the line between the first two
	if(addSupportPoint())
		return false;


	// Find which side of the plane the origin is on
	direction_ = crossProduct(vertices_[1].v - vertices_[0].v, vertices_[2].v - vertices_[0].v);

	// Reverse plane if necessary
	if(direction_ * vertices_[0].v > 0){
		CVertex tmp = vertices_[1];
		vertices_[1] = vertices_[2];
		vertices_[2] = tmp;

		direction_ *= -1;
	}

	// Fourth point (forming first triangular portal) in direction of plane normal
	if(addSupportPoint())
		return false;


	// Check that the origin is enclosed and adjust points accordingly
	for(int i = 0; i < 9; i++){
		if(crossProduct(vertices_[1].v, vertices_[3].v) * vertices_[0].v <= 0){
			vertices_[2] = vertices_[3];
			vertices_.erase(vertices_.begin() + 3);

			direction_ = crossProduct(vertices_[1].v - vertices_[0].v, vertices_[2].v - vertices_[0].v);

			if(addSupportPoint())
				return false;

			continue;
		}
		if(crossProduct(vertices_[3].v, vertices_[2].v) * vertices_[0].v <= 0){
			vertices_[1] = vertices_[3];
			vertices_.erase(vertices_.begin() + 3);

			direction_ = crossProduct(vertices_[2].v - vertices_[0].v, vertices_[1].v - vertices_[0].v);

			if(addSupportPoint())
				return false;

			continue;
		}
		break;
	}

	// Refine portal
	for(int i = 0; i < PHYS_MPR_MAX_ITER; i++){

		// Check if origin lies on the same side of the portal as the interior vertex
		direction_ = crossProduct(vertices_[2].v - vertices_[1].v, vertices_[3].v - vertices_[1].v);

		// Collision found
		if(direction_ * vertices_[1].v >= 0){
			return true;
		}

		// Add new point in direction of portal normal, creating tetrahedron
		if(addSupportPoint())
			return false;


		// Convergence test
		if((vertices_[4].v - vertices_[3].v) * direction_ <= 0.0001f || vertices_[4].v * direction_ <= 0)
			return false;

		// Test which tetahedron face the origin ray passes through
		float d1 = crossProduct(vertices_[4].v, vertices_[1].v) * vertices_[0].v;
		float d2 = crossProduct(vertices_[4].v, vertices_[2].v) * vertices_[0].v;
		float d3 = crossProduct(vertices_[4].v, vertices_[3].v) * vertices_[0].v;

		// Get vertex to replace
		int vertex = 0;
		if(d1 < 0){
			if(d2 < 0)	vertex = 1;
			else		vertex = 3;
		}
		else{
			if(d3 < 0)	vertex = 2;
			else		vertex = 1;
		}

		// Replace and eliminate vertex
		vertices_[vertex] = vertices_[4];
		vertices_.erase(vertices_.begin() + 4);
	}
	
	// No collision found within iteration limit
	ntw::warning("MPR failed within iteration limit!");
	return false;
}

bool MPRCollision::addSupportPoint(){
	CVertex vertex = getSupportPoint();
	vertices_.push_back(vertex);
	return vertex.v * direction_ <= 0;
}

CVertex MPRCollision::getSupportPoint(){
	direction_.normalize();
	CVertex vertex;
	vertex.vObj1 = getFurthestPoint(true, -direction_);
	vertex.vObj2 = getFurthestPoint(false, direction_);
	vertex.v = vertex.vObj2 - vertex.vObj1;

	return vertex;
}

Vec3 MPRCollision::getFurthestPoint(const bool& useObject1, const Vec3& direction){

	// Use transformed hitbox vertices if they exist
	vector<Vec3>& verts = useObject1 ? object1Verts_ : object2Verts_;

	if(!verts.empty()){
		Vec3 vertex =  verts[0];

		for(auto i = verts.begin(); i != verts.end(); i++)
			if((*i - vertex) * direction > 0)
				vertex = *i;

		return vertex;
	}


	// If no vertices were cached
	Object*& obj = useObject1 ? object1_ : object2_;

	Vec3 position = obj->getPosition();
	Vec3 scale = obj->getScale();
	Quaternion rotation = obj->getRotation();

	Matrix transform;

	/*
	switch(obj->getHitboxType()){
	}
	*/

	return Vec3();
}


Contact MPRCollision::getContact(){
	
	/*
	if(vertices_.size() == 4){
		Vec3 n1 = crossProduct(vertices_[1].v - vertices_[0].v, vertices_[2].v - vertices_[0].v);
		Vec3 n2 = crossProduct(vertices_[2].v - vertices_[1].v, vertices_[3].v - vertices_[1].v);
		Vec3 n3 = crossProduct(vertices_[3].v - vertices_[2].v, vertices_[0].v - vertices_[2].v);
		Vec3 n4 = crossProduct(vertices_[0].v - vertices_[3].v, vertices_[1].v - vertices_[3].v);

		if(
			((n1 * (vertices_[3].v - vertices_[0].v) > 0) != (n1 * -vertices_[0].v > 0)) &&
			((n2 * (vertices_[0].v - vertices_[1].v) > 0) != (n2 * -vertices_[1].v > 0)) &&
			((n3 * (vertices_[1].v - vertices_[2].v) > 0) != (n3 * -vertices_[2].v > 0)) &&
			((n4 * (vertices_[2].v - vertices_[3].v) > 0) != (n4 * -vertices_[3].v > 0))
			)
			ntw::error("ORIGIN IS NOT IN TETRAHEDRON !!!");
	}
	else
		ntw::error("EPA started with " + std::to_string(vertices_.size()) + " vertices");
	*/

	// If the collision was resolved using a line segment, return the point closer to the origin
	if(vertices_.size() == 2){
		direction_ = vertices_[0].v.magnitude() < vertices_[1].v.magnitude() ? vertices_[0].v : vertices_[1].v;

		// Calculate support point to get contact points
		CVertex v = getSupportPoint();

		Contact c;
		c.penetration = direction_;
		c.obj1ContactGlobal = v.vObj1;
		c.obj2ContactGlobal = v.vObj2;
		setContactInfo(c);

		return c;
	}

	// If it was a triangle, create two tetrahedrons
	else if(vertices_.size() == 3){
		direction_ = crossProduct(vertices_[1].v - vertices_[0].v, vertices_[2].v - vertices_[0].v);
		addSupportPoint();
		direction_ *= -1;
		addSupportPoint();

		// Create face list, normals facing away from origin
		addFace(vertices_[2], vertices_[1], vertices_[0]);
		addFace(vertices_[0], vertices_[1], vertices_[3]);
		addFace(vertices_[3], vertices_[2], vertices_[0]);
		addFace(vertices_[1], vertices_[2], vertices_[3]);

		addFace(vertices_[2], vertices_[1], vertices_[0]);
		addFace(vertices_[0], vertices_[1], vertices_[4]);
		addFace(vertices_[4], vertices_[2], vertices_[0]);
		addFace(vertices_[1], vertices_[2], vertices_[4]);
	}

	else{
		// Replace interior vertex using face normal of the other 3
		direction_ = crossProduct(vertices_[2].v - vertices_[1].v, vertices_[3].v - vertices_[1].v);

		// Make sure the tetrahedron encloses the origin
		if(direction_ * vertices_[1].v >= 0)
			direction_ = -direction_;

		// Replace interior
		vertices_[0] = getSupportPoint();

		// Create face list, normals facing away from origin
		addFace(vertices_[2], vertices_[1], vertices_[0]);
		addFace(vertices_[0], vertices_[1], vertices_[3]);
		addFace(vertices_[3], vertices_[2], vertices_[0]);
		addFace(vertices_[1], vertices_[2], vertices_[3]);
	}

	int i = 0;

	while(true){

		// Get currently present face that is closest to the origin
		int faceIndex = getFaceClosestToOrigin();
		EPAFace face = faces_[faceIndex];

		// Get support point in direction of face normal
		direction_ = face.normal;
		CVertex support = getSupportPoint();

		// Get support vector distance and penetration vector
		float distance = support.v * direction_;

		// Closest face found within tolerance or iteration limit hit, return penetration
		if(abs(distance - face.distance) < 0.001f || /*abs(face.distance) < PHYS_EPA_THRESHOLD ||*/ i >= PHYS_EPA_MAX_ITER){

			//if(i >= PHYS_EPA_MAX_ITER)
			//	ntw::warning("EPA failed within iteration limit!");

			// Penetration vector
			Vec3 penetration = face.normal * face.distance;

			// Get barycentric coordinates of origin on closest triangle
			Vec3 v0 = face.v2.v - face.v1.v;
			Vec3 v1 = face.v3.v - face.v1.v;
			Vec3 v2 = penetration - face.v1.v;

			float d00 = v0.magnitude2();
			float d01 = v0 * v1;
			float d11 = v1.magnitude2();
			float d20 = v2 * v0;
			float d21 = v2 * v1;
			float denom = d00 * d11 - d01 * d01;

			float v = (d11 * d20 - d01 * d21) / denom;
			float w = (d00 * d21 - d01 * d20) / denom;
			float u = 1 - v - w;

			// Multiply values with support points of each object to get contact points
			/*
			vector<Vec3> c;

			// Add object support points
			c.push_back(getFurthestPoint(true,	faceVerts[2]));
			c.push_back(getFurthestPoint(true,	faceVerts[1]));
			c.push_back(getFurthestPoint(true,	faceVerts[0]));
			c.push_back(getFurthestPoint(false,	faceVerts[0]));
			c.push_back(getFurthestPoint(false,	faceVerts[1]));
			c.push_back(getFurthestPoint(false,	faceVerts[2]));
			*/

			Contact c;
			c.penetration = penetration;

			// Multiply values with support points of each object to get contact points
			c.obj1ContactGlobal	= u * face.v1.vObj1 + v * face.v2.vObj1 + w * face.v3.vObj1;
			c.obj2ContactGlobal	= u * face.v1.vObj2 + v * face.v2.vObj2 + w * face.v3.vObj2;
			setContactInfo(c);

			return c;
		}

		// Find faces that should be removed
		edges_.clear();
		for(int j = 0; j < faces_.size(); j++){
			
			EPAFace& face = faces_[j];

			// Remove if it can be seen by support point
			if(face.normal * (support.v - face.v1.v) > 0){
				// Keep track of face edges
				addEdge(face.v1, face.v2);
				addEdge(face.v2, face.v3);
				addEdge(face.v3, face.v1);

				// Remove face
				faces_.erase(faces_.begin() + j);
				j--;
			}
		}

		// Create new faces
		for(auto j = edges_.begin(); j != edges_.end(); j++)
			addFace(support, (*j).v1, (*j).v2);

		i++;
		/*
		// Split face
		// Start by removing current face
		faces_.erase(faces_.begin() + faceIndex);

		// Split edges of removed face
		for(int j = 0; j < 3; j++){

			// Get second edge vertex
			int k = j == 2 ? 0 : j + 1;

			CVertex& v1 = j == 0 ? face.v1 : j == 1 ? face.v2 : face.v3;
			CVertex& v2 = k == 0 ? face.v1 : k == 1 ? face.v2 : face.v3;

			// Add intermediate face
			addFace(v1, v2, support);

			// There should be code here to optimize the algorithm slightly,
			// but it doesn't work and doesn't seem necessary
			/*
			// Get point on edge closest to the origin, and the support point in that direction
			direction_ = faceVerts[k] - faceVerts[j];
			direction_ = faceVerts[j] + direction_ * (-faceVerts[j] * direction_);
			Vec3 edgeSupport = getSupportPoint();

			// If current edge is part of Minkowski difference boundary, it does not need to be split
			if(direction_ == edgeSupport){
				continue;
			}

			// Add faces
			addFace(support, faceVerts[j], edgeSupport);
			addFace(support, faceVerts[k], edgeSupport);
			*\/
		}
		*/
	}
}

void MPRCollision::addFace(const CVertex& v1, const CVertex& v2, const CVertex& v3){

	// Create face
	EPAFace face;
	face.v1 = v1;
	face.v2 = v2;
	face.v3 = v3;

	// Compute normal and check that it faces away from the origin
	face.normal = crossProduct(v2.v - v1.v, v3.v - v1.v).normalize();

	// Bias to account for inaccuracy
	if(face.normal * v1.v + 0.000001 <= 0){
		face.normal *= -1;
		face.v1 = v3;
		face.v3 = v1;
	}

	// Distance to origin
	face.distance = abs(face.normal * face.v1.v);

	faces_.push_back(face);
}

void MPRCollision::addEdge(const CVertex& v1, const CVertex& v2){

	// Check if there is an opposite edge
	for(auto i = edges_.begin(); i != edges_.end(); i++){

		// If there is, remove it and do not add the new edge
		if((*i).v1 == v2 && (*i).v2 == v1){
			edges_.erase(i);
			return;
		}
	}

	// No conflicting edge
	edges_.push_back({v1, v2});
}

int MPRCollision::getFaceClosestToOrigin(){

	float minDistance = std::numeric_limits<float>::max();
	int index = 0;

	// Loop through triangular faces
	for(auto i = faces_.begin(); i != faces_.end(); i++){

		// Check distance
		if((*i).distance < minDistance){
			minDistance = (*i).distance;
			index = (int)(i - faces_.begin());
		}
	}

	return index;
}

void MPRCollision::setContactInfo(Contact& c){

	// Contact vector
	c.obj1ContactVector = c.obj1ContactGlobal - obj1Pos_;
	c.obj2ContactVector = c.obj2ContactGlobal - obj2Pos_;
	
	// Local contacts
	c.obj1ContactLocal = obj1Rotation_.getTranspose() * c.obj1ContactVector;
	c.obj2ContactLocal = obj2Rotation_.getTranspose() * c.obj2ContactVector;
	

	// Contact tangents
	Vec3 normal = c.penetration.unitVector();

	if(normal[0] >= 0.57735f)
		c.tangent1 = Vec3(normal[1], -normal[0], 0);
	else
		c.tangent1 = Vec3(0, normal[2], -normal[1]);

	c.tangent2 = crossProduct(normal, c.tangent1);
}
