#include"SATCollision.h"

#include"core/error.h"
#include"math/matrix.h"
#include"physDefine.h"
#include<algorithm>
#include<limits>
#include<math.h>

using ntw::crossProduct;
using std::min;
using std::max;


SATCollision::SATCollision(Object* object1, Object* object2) : object1_(object1), object2_(object2),
	obj1Hitbox_(object1->getTransformedHitboxSAT()), obj2Hitbox_(object2->getTransformedHitboxSAT()) {
	
}

bool SATCollision::testCollision(){
	
	// Initialize contact point info to get shortest distance between features
	contactInfo_ = {-std::numeric_limits<float>::max(), false, -1, -1};

	// Distance checks
	if(queryFaces(obj1Hitbox_, obj2Hitbox_, true) > NTW_SAT_THRESHOLD)
		return false;

	if(queryFaces(obj2Hitbox_, obj1Hitbox_, false) > NTW_SAT_THRESHOLD)
		return false;
	
	if(queryEdges(obj1Hitbox_, obj2Hitbox_) > NTW_SAT_THRESHOLD)
		return false;

	return true;
}

ContactManifold SATCollision::getContactPoints(){

	// Contact points on edges
	if(contactInfo_.isEdgePair){

		// Edges
		const SATHalfEdge& e1 = obj1Hitbox_.edges[contactInfo_.index1];
		const SATHalfEdge& e2 = obj2Hitbox_.edges[contactInfo_.index2];

		// Edge start points
		Vec3 p1 = obj1Hitbox_.vertices[e1.v1];
		Vec3 p2 = obj2Hitbox_.vertices[e2.v1];

		// Edge directions
		Vec3 d1 = obj1Hitbox_.vertices[e1.v2] - obj1Hitbox_.vertices[e1.v1];
		Vec3 d2 = obj2Hitbox_.vertices[e2.v2] - obj2Hitbox_.vertices[e2.v1];

		// Normals
		Vec3 n = crossProduct(d1, d2);
		Vec3 n1 = crossProduct(d1, n);
		Vec3 n2 = crossProduct(d2, n);

		// Contact points are closest points on each edge
		Contact c;
		c.obj1ContactGlobal = p1 + ((((p2 - p1) * n2) / (d1 * n2)) * d1);
		c.obj2ContactGlobal = p2 + ((((p1 - p2) * n1) / (d2 * n1)) * d2);

		c.normal = (c.obj2ContactGlobal - c.obj1ContactGlobal).unitVector();
		setContactInfo(c);

		// Create manifold and return
		ContactManifold m;
		m.objects = {object1_, object2_};
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
		normal = -obj1Hitbox_.faces[fi1].normal;

		// Get opposite face
		float bestDot = std::numeric_limits<float>::max();

		for(int i = 0; i < obj2Hitbox_.faces.size(); i++){
			float dot = obj2Hitbox_.faces[i].normal* obj1Hitbox_.faces[fi1].normal;

			if(dot < bestDot){
				fi2 = i;
				bestDot = dot;
			}
		}
	}
	else if(fi2 != -1){
		normal = obj2Hitbox_.faces[fi2].normal;

		// Get opposite face
		float bestDot = std::numeric_limits<float>::max();

		for(int i = 0; i < obj1Hitbox_.faces.size(); i++){
			float dot = obj1Hitbox_.faces[i].normal * obj2Hitbox_.faces[fi2].normal;

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
	const SATFace& f1 = obj1Hitbox_.faces[fi1];
	const SATFace& f2 = obj2Hitbox_.faces[fi2];

	//normal = (f2.normal.unitVector() - f1.normal.unitVector()) / 2;

	// Clip faces to get single contact points
	vector<Vec3> points = clipFaces(f1, f2);

	// Create manifold
	ContactManifold m;
	m.objects = {object1_, object2_};
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
		if(setContactInfo(c))
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
		if(distance > maxDistance)
			maxDistance = distance;

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
				float distance = getEdgeToEdgeDistance(e1, e2);

				// Largest distance
				if(distance > maxDistance)
					maxDistance = distance;

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

float SATCollision::getFaceToPointDistance(const SATFace& f, const Vec3& v){
	return f.normal * (v - f.position);
}


SATInterval SATCollision::project(const Hitbox& hitbox, const Vec3& axis){

	float d = hitbox.vertices[0] * axis;
	SATInterval i = {d, d};

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

float SATCollision::getEdgeToEdgeDistance(const SATHalfEdge& e1, const SATHalfEdge& e2){
	
	Vec3 cross = crossProduct(obj1Hitbox_.vertices[e1.v1] - obj1Hitbox_.vertices[e1.v2], obj2Hitbox_.vertices[e2.v1] - obj2Hitbox_.vertices[e2.v2]);

	// Ignore parallel edges
	if(cross.magnitude2() < 0.00001f)
		return -std::numeric_limits<float>::max();
	

	cross.normalize();

	// Check normal direction
	if(cross * (obj1Hitbox_.vertices[e1.v1] - object1_->getTPosition()) < 0)
		cross = -cross;

	return cross * (obj2Hitbox_.vertices[e2.v1] - obj1Hitbox_.vertices[e1.v1]);
}

vector<Vec3> SATCollision::clipFaces(const SATFace& f1, const SATFace& f2){

	// Get clipping planes
	vector<SATFace> clippingPlanes;

	for(int edgeIndex : f1.edges){

		// Current edge
		const SATHalfEdge& e = obj1Hitbox_.edges[edgeIndex];

		// Create clipping plane
		SATFace plane;
		plane.position = obj1Hitbox_.vertices[e.v1];
		plane.normal = crossProduct(obj1Hitbox_.vertices[e.v2] - plane.position, f1.normal);

		// Check normal direction, normal should face towards center of original face
		if(plane.normal * (f1.position - plane.position) < 0)
			plane.normal = -plane.normal;

		clippingPlanes.push_back(plane);
	}

	// Points to clip
	vector<Vec3> points;


	// Make a copy of second face edges
	vector<SATHalfEdge> f2Edges;

	for(int edgeIndex : f2.edges)
		f2Edges.push_back(obj2Hitbox_.edges[edgeIndex]);

	// Get vertices of second face, adding them in a cyclic order
	while(!f2Edges.empty()){

		// Loop through remaining edges, adding one when its vertices overlap with the last added vertex
		for(auto i = f2Edges.begin(); i != f2Edges.end(); i++){

			// Edge vertices
			const Vec3& v1 = obj2Hitbox_.vertices[(*i).v1];
			const Vec3& v2 = obj2Hitbox_.vertices[(*i).v2];

			// Initial points
			if(points.empty()){
				points.push_back(v1);
				points.push_back(v2);
				f2Edges.erase(i);
				break;
			}

			// v1 overlaps with last vertex and v2 not present in list
			if(v1.equalsWithinThreshold(points[points.size() - 1], 0.00001f)){
				if(std::find(points.begin(), points.end(), v2) == points.end())
					points.push_back(v2);
				f2Edges.erase(i);
				break;
			}

			// v2 overlaps with last vertex and v1 not present in list
			else if(v2.equalsWithinThreshold(points[points.size() - 1], 0.00001f)){
				if(std::find(points.begin(), points.end(), v1) == points.end())
					points.push_back(v1);
				f2Edges.erase(i);
				break;
			}
		}
	}

	// Clipping
	for(const SATFace& plane : clippingPlanes){

		// Clipped points
		vector<Vec3> output;

		// Clip each pair of points
		for(int i = 0; i < points.size(); i++){

			// Second point index
			int j = i + 1;
			j = j >= points.size() ? 0 : j;

			// Points
			Vec3 v1 = points[i];
			Vec3 v2 = points[j];

			// Which side of clipping plane the points are on
			bool v1Front = getFaceToPointDistance(plane, v1) > 0;
			bool v2Front = getFaceToPointDistance(plane, v2) > 0;


			// Function to get point of intersection between line and plane
			auto l_intersect = [](const Vec3& v1, const Vec3& v2, const SATFace& f){
				Vec3 dir = v2 - v1;
				return v1 + ((((f.position - v1) * f.normal) / (dir * f.normal)) * dir);
			};


			// End point in front
			if(v2Front){

				// If the line segment crosses the clipping plane, add intersecting point
				if(!v1Front)
					output.push_back(l_intersect(v1, v2, plane));

				output.push_back(v2);
			}

			// Start point in front, end point behind
			else if(v1Front)
				output.push_back(l_intersect(v1, v2, plane));
				
		}

		points = output;
	}

	if(points.size() == 0)
		ntw::warning("SAT face clipping did not generate any points!");

	return points;
}

bool SATCollision::setContactInfo(Contact& c){

	// Normal and penetration depth
	//Vec3 diff = c.obj2ContactGlobal - c.obj1ContactGlobal;
	//c.depth = diff.magnitude();
	//c.normal = diff / c.depth;

	c.depth = (c.obj2ContactGlobal - c.obj1ContactGlobal) * c.normal;

	// Check validity
	if(c.depth < 0)
		return false;


	// Contact vector
	c.obj1ContactVector = c.obj1ContactGlobal - object1_->getTPosition();
	c.obj2ContactVector = c.obj2ContactGlobal - object2_->getTPosition();

	// Contact tangents
	if(c.normal[0] >= 0.57735f)
		c.tangent1 = Vec3(c.normal[1], -c.normal[0], 0);
	else
		c.tangent1 = Vec3(0, c.normal[2], -c.normal[1]);

	c.tangent1.normalize();
	c.tangent2 = crossProduct(c.normal, c.tangent1);

	return true;
}
