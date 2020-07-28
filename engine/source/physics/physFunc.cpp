#include"physFunc.h"

#include"core/error.h"




float ntw::getFaceToPointDistance(const SATFace& f, const Vec3& v){
	return f.normal * (v - f.position);
}

float ntw::getEdgeToEdgeDistance(const Hitbox& hitbox1, int edgeIndex1, const Hitbox& hitbox2, int edgeIndex2){

	const SATHalfEdge& e1 = hitbox1.edges[edgeIndex1];
	const SATHalfEdge& e2 = hitbox2.edges[edgeIndex2];

	Vec3 cross = ntw::crossProduct(hitbox1.vertices[e1.v1] - hitbox1.vertices[e1.v2], hitbox2.vertices[e2.v1] - hitbox2.vertices[e2.v2]);

	// Ignore parallel edges
	if(cross.magnitude2() < 0.00001f)
		return -std::numeric_limits<float>::max();


	cross.normalize();

	// Get hitbox center
	Vec3 center;

	for(const Vec3& v : hitbox1.vertices)
		center += v;

	center /= (float)hitbox1.vertices.size();

	// Check normal direction
	if(cross * (hitbox1.vertices[e1.v1] - center) < 0)
		cross = -cross;

	return cross * (hitbox2.vertices[e2.v1] - hitbox1.vertices[e1.v1]);
}


vector<SATFace> ntw::getClippingPlanes(const Hitbox& hitbox, int faceIndex){

	const SATFace& face = hitbox.faces[faceIndex];
	vector<SATFace> clippingPlanes;

	for(int edgeIndex : face.edges){

		// Current edge
		const SATHalfEdge& e = hitbox.edges[edgeIndex];

		// Create clipping plane
		SATFace plane;
		plane.position = hitbox.vertices[e.v1];
		plane.normal = ntw::crossProduct(hitbox.vertices[e.v2] - plane.position, face.normal);

		// Check normal direction, normal should face towards center of original face
		if(plane.normal * (face.position - plane.position) < 0)
			plane.normal = -plane.normal;

		clippingPlanes.push_back(plane);
	}

	return clippingPlanes;
}

vector<Vec3> ntw::clipFaces(const Hitbox& hitbox1, int faceIndex1, const Hitbox& hitbox2, int faceIndex2){

	const SATFace& f1 = hitbox1.faces[faceIndex1];
	const SATFace& f2 = hitbox2.faces[faceIndex2];

	// Get clipping planes
	vector<SATFace> clippingPlanes = ntw::getClippingPlanes(hitbox1, faceIndex1);

	// Points to clip
	vector<Vec3> points;

	// Make a copy of second face edges
	vector<SATHalfEdge> f2Edges;

	for(int edgeIndex : f2.edges)
		f2Edges.push_back(hitbox2.edges[edgeIndex]);


	// Get vertices of second face, adding them in a cyclic order
	while(!f2Edges.empty()){

		// Loop through remaining edges, adding one when its vertices overlap with the last added vertex
		for(auto i = f2Edges.begin(); i != f2Edges.end(); i++){

			// Edge vertices
			const Vec3& v1 = hitbox2.vertices[(*i).v1];
			const Vec3& v2 = hitbox2.vertices[(*i).v2];

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

	return points;
}

float ntw::raycast(const Vec3& rayPosition, const Vec3& rayDirection, float maxDistance, const Hitbox& hitbox){

	bool inside = true;

	for(int i = 0; i < hitbox.faces.size(); i++){

		const SATFace& face = hitbox.faces[i];

		// Check case where ray begins inside hitbox
		if(inside && ntw::getFaceToPointDistance(face, rayPosition) < 0)
			inside = false;

		// Check only faces with normals opposite to the ray (closest faces on hitbox)
		if(face.normal * rayDirection > 0)
			continue;

		// Find point where ray intersects face
		float d = ((face.position - rayPosition) * face.normal) / (rayDirection * face.normal);

		// Discard intersections behind ray position and further than maximum distance
		if(d <= 0 || d > maxDistance)
			continue;

		// Intersection point
		Vec3 p = rayPosition + rayDirection * d;

		// Check that point is on face by checking against face clipping planes
		vector<SATFace> clippingPlanes = ntw::getClippingPlanes(hitbox, i);

		for(const SATFace& plane : clippingPlanes)
			if(ntw::getFaceToPointDistance(plane, p) < 0)
				goto faceLoop;


		// Intersection found, return distance
		return d;

	faceLoop:;
	}

	// Ray cast from inside hitbox, return distance 0
	if(inside)
		return 0;

	return -1;
}
