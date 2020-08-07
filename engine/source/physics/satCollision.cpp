#include"SATCollision.h"

#include"core/error.h"
#include"math/matrix.h"
#include"physics/physDefine.h"
#include"physics/physFunc.h"
#include<algorithm>
#include<limits>
#include<math.h>

using ntw::crossProduct;
using ntw::getFaceToPointDistance;
using ntw::getEdgeToEdgeDistance;
using ntw::clipFaces;
using std::min;
using std::max;


SATCollision::SATCollision(const Collider* collider1, const Collider* collider2) : collider1_(collider1), collider2_(collider2),
	hitbox1_(collider1->hitboxTransformed), hitbox2_(collider2->hitboxTransformed) {
	
}

bool SATCollision::testCollision(){
	
	// Initialize contact point info to get shortest distance between features
	contactInfo_ = {-std::numeric_limits<float>::max(), false, -1, -1};

	// Distance checks
	if(queryFaces(hitbox1_, hitbox2_, true) > NTW_SAT_THRESHOLD)
		return false;

	if(queryFaces(hitbox2_, hitbox1_, false) > NTW_SAT_THRESHOLD)
		return false;
	
	if(queryEdges(hitbox1_, hitbox2_) > NTW_SAT_THRESHOLD)
		return false;

	return true;
}

bool SATCollision::testCollision(const SATSeparatingAxis& axis){
	
	// Check collision with previously found axis
	if(!axis.isEdgePair){
		const SATFace& f = axis.index1 != -1 ? hitbox1_.faces[axis.index1] : hitbox2_.faces[axis.index2];
		Vec3 support = getSupportPoint(axis.index1 != -1 ? hitbox2_ : hitbox1_, -f.normal);

		// Largest distance
		if(getFaceToPointDistance(f, support) > NTW_SAT_THRESHOLD)
			return false;
	}
	else if(getEdgeToEdgeDistance(hitbox1_, axis.index1, hitbox2_, axis.index2) > NTW_SAT_THRESHOLD)
		return false;

	// Separating axis is invalid, perform full SAT test
	return testCollision();
}

ContactManifold SATCollision::getContactPoints(){

	// Contact points on edges
	if(contactInfo_.isEdgePair){

		// Edges
		const SATHalfEdge& e1 = hitbox1_.edges[contactInfo_.index1];
		const SATHalfEdge& e2 = hitbox2_.edges[contactInfo_.index2];

		// Edge start points
		Vec3 p1 = hitbox1_.vertices[e1.v1];
		Vec3 p2 = hitbox2_.vertices[e2.v1];

		// Edge directions
		Vec3 d1 = hitbox1_.vertices[e1.v2] - hitbox1_.vertices[e1.v1];
		Vec3 d2 = hitbox2_.vertices[e2.v2] - hitbox2_.vertices[e2.v1];

		// Normals
		Vec3 n = crossProduct(d1, d2);
		Vec3 n1 = crossProduct(d1, n);
		Vec3 n2 = crossProduct(d2, n);

		// Contact points are closest points on each edge
		Contact c;
		c.obj1ContactGlobal = p1 + ((((p2 - p1) * n2) / (d1 * n2)) * d1);
		c.obj2ContactGlobal = p2 + ((((p1 - p2) * n1) / (d2 * n1)) * d2);

		c.normal = (c.obj2ContactGlobal - c.obj1ContactGlobal).unitVector();
		setContactProperties(c);

		// Create manifold and return
		ContactManifold m;
		m.objects = {collider1_->parent, collider2_->parent};
		m.maxDistance = -contactInfo_.distance;
		m.contacts.push_back(c);

		return m;
	}


	// Get indices of faces to clip
	int fi1 = contactInfo_.index1;
	int fi2 = contactInfo_.index2;

	Vec3 biasDirection = Vec3(0, 0, 1);

	// Contact normal
	Vec3 normal;


	if(fi1 != -1){
		normal = -hitbox1_.faces[fi1].normal;

		// Get opposite face
		float bestDot = std::numeric_limits<float>::max();

		for(int i = 0; i < hitbox2_.faces.size(); i++){
			float dot = hitbox2_.faces[i].normal* hitbox1_.faces[fi1].normal;

			if(dot < bestDot){
				fi2 = i;
				bestDot = dot;
			}
		}
	}
	else if(fi2 != -1){
		normal = hitbox2_.faces[fi2].normal;

		// Get opposite face
		float bestDot = std::numeric_limits<float>::max();

		for(int i = 0; i < hitbox1_.faces.size(); i++){
			float dot = hitbox1_.faces[i].normal * hitbox2_.faces[fi2].normal;

			if(dot < bestDot){
				fi1 = i;
				bestDot = dot;
			}
		}
	}
	else{
		ntw::warning("SAT contact point generation failure, edge did not have any adjacent faces");
		return ContactManifold();
	}

	// Get faces
	const SATFace& f1 = hitbox1_.faces[fi1];
	const SATFace& f2 = hitbox2_.faces[fi2];

	// Clip faces to get single contact points
	vector<Vec3> points = clipFaces(hitbox1_, fi1, hitbox2_, fi2);

	// Create manifold
	ContactManifold m;
	m.objects = {collider1_->parent, collider2_->parent};
	m.maxDistance = -contactInfo_.distance;
	
	for(const Vec3& v : points){

		Contact c;

		// Contact points for each object are clipped points projected onto each object's respective face
		auto l_project = [](const Vec3& v, const SATFace& f){
			const Vec3& p = f.position;
			const Vec3& n = f.normal;
			return v + (n * (((n * p) - (n * v)) / n.magnitude2()));
		};

		c.obj1ContactGlobal = l_project(v, f1);
		c.obj2ContactGlobal = l_project(v, f2);
		c.normal = normal;

		// Set contact properties and check its validity before adding
		if(setContactProperties(c))
			m.contacts.push_back(c);
	}

	return m;
}


float SATCollision::queryFaces(const Hitbox& hitbox1, const Hitbox& hitbox2, bool useIndex1){

	float maxDistance = -std::numeric_limits<float>::max();

	// Loop through faces
	for(int i = 0; i < hitbox1.faces.size(); i++){

		const SATFace& f = hitbox1.faces[i];
		Vec3 support = getSupportPoint(hitbox2, -f.normal);
		float distance = getFaceToPointDistance(f, support);

		// Largest distance
		if(distance > maxDistance){
			maxDistance = distance;

			// Set separating axis info
			separatingAxis_.isEdgePair = false;
			separatingAxis_.index1 = useIndex1 ? i : -1;
			separatingAxis_.index2 = !useIndex1 ? i : -1;
		}

		// Smallest penetration distance over all features
		if(distance < 0 && distance > contactInfo_.distance){

			// Set contact info
			contactInfo_.distance = distance;
			contactInfo_.isEdgePair = false;

			if(useIndex1){
				contactInfo_.index1 = i;
				contactInfo_.index2 = -1;
			}
			else{
				contactInfo_.index1 = -1;
				contactInfo_.index2 = i;
			}
		}
	}

	return maxDistance;
}

float SATCollision::queryEdges(const Hitbox& hitbox1, const Hitbox& hitbox2){

	float maxDistance = -std::numeric_limits<float>::max();

	// Loop through edge pairs
	for(int i = 0; i < hitbox1.edges.size(); i++){
		for(int j = 0; j < hitbox2.edges.size(); j++){

			const SATHalfEdge& e1 = hitbox1.edges[i];
			const SATHalfEdge& e2 = hitbox2.edges[j];

			// Check for invalid edge faces
			if(e1.f1 == -1 || e1.f2 == -1 || e2.f1 == -1 || e2.f2 == -1)
				continue;

			// Edge pruning
			if(isMinkowskiFace(hitbox1.faces[e1.f1].normal, hitbox1.faces[e1.f2].normal, -hitbox2.faces[e2.f1].normal, -hitbox2.faces[e2.f2].normal)){
				float distance = getEdgeToEdgeDistance(hitbox1, i, hitbox2, j);

				// Largest distance
				if(distance > maxDistance){
					maxDistance = distance;

					// Set separating axis info
					separatingAxis_.isEdgePair = true;
					separatingAxis_.index1 = i;
					separatingAxis_.index2 = j;
				}

				// Smallest penetration distance over all features
				if(distance < 0 && distance > contactInfo_.distance){

					// Set contact info
					contactInfo_.distance = distance;
					contactInfo_.isEdgePair = true;
					contactInfo_.index1 = i;
					contactInfo_.index2 = j;
				}
			}
		}
	}

	return maxDistance;
}

Vec3 SATCollision::getSupportPoint(const Hitbox& hitbox, const Vec3& direction){

	Vec3 vertex = hitbox.vertices[0];
	float maxProduct = direction * vertex;

	for(int i = 1; i < hitbox.vertices.size(); i++){
		const Vec3& v = hitbox.vertices[i];
		float product = direction * v;

		if(product > maxProduct){
			vertex = v;
			maxProduct = product;
		}
	}

	return vertex;
}


SATCollision::EdgeInterval SATCollision::project(const Hitbox& hitbox, const Vec3& axis){

	float d = hitbox.vertices[0] * axis;
	EdgeInterval i = {d, d};

	// Get smallest and largest dot product
	for(const Vec3& v : hitbox.vertices){
		d = v * axis;
		i.v1 = min(i.v1, d);
		i.v2 = max(i.v2, d);
	}

	return i;
}

bool SATCollision::isMinkowskiFace(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d){

	Vec3 bxa = crossProduct(b, a);
	Vec3 dxc = crossProduct(d, c);

	float cba = c * bxa;
	float dba = d * bxa;
	float adc = a * dxc;
	float bdc = b * dxc;

	return cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0;
}

bool SATCollision::setContactProperties(Contact& c){

	c.depth = (c.obj2ContactGlobal - c.obj1ContactGlobal) * c.normal;

	// Check validity
	if(c.depth < 0)
		return false;


	// Contact vector
	c.obj1ContactVector = c.obj1ContactGlobal - collider1_->parent->getTPosition();
	c.obj2ContactVector = c.obj2ContactGlobal - collider2_->parent->getTPosition();

	// Contact tangents
	if(c.normal[0] >= 0.57735f)
		c.tangent1 = Vec3(c.normal[1], -c.normal[0], 0);
	else
		c.tangent1 = Vec3(0, c.normal[2], -c.normal[1]);

	c.tangent1.normalize();
	c.tangent2 = crossProduct(c.normal, c.tangent1);

	return true;
}

void SATCollision::setContactInfo(const SATContactInfo& contactInfo){
	contactInfo_ = contactInfo;
}

SATSeparatingAxis SATCollision::getSeparatingAxis(){
	return separatingAxis_;
}

SATContactInfo SATCollision::getContactInfo(){
	return contactInfo_;
}
