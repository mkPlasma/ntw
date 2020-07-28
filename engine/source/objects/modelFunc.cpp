#include"modelFunc.h"

#include"math/mathFunc.h"
#include"core/error.h"
#include<algorithm>


#define addVertex(v) vertices.push_back(v[0]); vertices.push_back(v[1]); vertices.push_back(v[2])


void ntw::generateHitbox(Model* model){

	// Vertices to generate hitbox from
	// TODO: add case for predefined hitbox later
	vector<float>& vertices = model->vertices;


	// Keep track of triangles to merge coplanar faces
	struct Triangle{
		Vec3 v1;
		Vec3 v2;
		Vec3 v3;

		bool isCoplanarWith(const Triangle& a){
			const float threshold = 0.000001f;
			Vec3 n = ntw::crossProduct(v2 - v1, v3 - v1);

			return	abs(n * (v1 - a.v1)) < threshold &&
				abs(n * (v1 - a.v2)) < threshold &&
				abs(n * (v1 - a.v3)) < threshold;
		}
	};

	vector<Triangle> tris;

	// Merged faces
	vector<vector<Vec3>> faces;

	// For each triangle
	for(size_t i = 0; i < vertices.size() / 9; i++){

		// Create triangle
		Triangle t{
			Vec3(vertices[i * 9],		vertices[i * 9 + 1],	vertices[i * 9 + 2]),
			Vec3(vertices[i * 9 + 3],	vertices[i * 9 + 4],	vertices[i * 9 + 5]),
			Vec3(vertices[i * 9 + 6],	vertices[i * 9 + 7],	vertices[i * 9 + 8])
		};

		// Check if it is coplanar with previous triangles
		for(int j = 0; j < tris.size(); j++){

			// If coplanar, merge face vertices
			if(t.isCoplanarWith(tris[j])){

				// Face to merge with
				vector<Vec3>& face = faces[j];

				// Number of vertices in this triangle that are unique to the face
				// Will be 1 for all valid cases
				int numUnique = 0;

				// Index of unique triangle vertex to add
				int uniqueIndex = -1;

				// Check uniqueness
				if(std::find(face.begin(), face.end(), t.v1) == face.end()){ numUnique++;	uniqueIndex = 1; }
				if(std::find(face.begin(), face.end(), t.v2) == face.end()){ numUnique++;	uniqueIndex = 2; }
				if(std::find(face.begin(), face.end(), t.v3) == face.end()){ numUnique++;	uniqueIndex = 3; }

				// Error check
				if(numUnique != 1)
					ntw::fatalError("Hitbox model has detached coplanar faces!");

				// Non-unique vertices
				const Vec3& v1 = uniqueIndex == 1 ? t.v3 : uniqueIndex == 2 ? t.v1 : t.v2;
				const Vec3& v2 = uniqueIndex == 1 ? t.v2 : uniqueIndex == 2 ? t.v3 : t.v1;

				// Unique vertex
				const Vec3& u = uniqueIndex == 1 ? t.v1 : uniqueIndex == 2 ? t.v2 : t.v3;


				// Find indices of non-unique vertices, unique vertex will be inserted between them
				auto v1index = std::find(face.begin(), face.end(), v1);
				auto v2index = std::find(face.begin(), face.end(), v2);

				// Index is first in vector
				if(v1index == face.begin()){
					if(v2index == v1index + 1)			face.insert(v2index, u);
					else if(v2index == face.end() - 1)	face.insert(face.end(), u);
				}
				// Index is last in vector
				else if(v1index == face.end() - 1){
					if(v2index == v1index - 1)			face.insert(v1index, u);
					else if(v2index == face.begin())	face.insert(face.begin(), u);
				}
				// Index is in between
				else{
					if(v2index == v1index + 1)			face.insert(v2index, u);
					else if(v2index == v1index - 1)		face.insert(v1index, u);
				}

				// Merge done, move to next triangle
				goto triLoop;
			}
		}

		// Not coplanar, add triangle and face
		tris.push_back(t);
		faces.push_back({t.v1, t.v2, t.v3});

	triLoop:;
	}

	// Create hitbox data
	Hitbox hitbox;

	for(const vector<Vec3>& face : faces){

		// Ignore faces with less than 3 vertices
		int numVerts = (int)face.size();

		if(numVerts < 3)
			continue;


		// Average vertex positions for face position
		Vec3 facePosition;

		// Add all unique vertices
		for(int i = 0; i < numVerts; i++)
			if(std::find(hitbox.vertices.begin(), hitbox.vertices.end(), face[i]) == hitbox.vertices.end())
				hitbox.vertices.push_back(face[i]);


		// Add half-edges
		for(int i = 0; i < numVerts; i++){

			// Add vertex position to average
			facePosition += face[i];

			// Get second edge vertex
			int j = i + 1;
			j = j >= numVerts ? 0 : j;

			// Get vertex indices
			int v1 = (int)(std::find(hitbox.vertices.begin(), hitbox.vertices.end(), face[i]) - hitbox.vertices.begin());
			int v2 = (int)(std::find(hitbox.vertices.begin(), hitbox.vertices.end(), face[j]) - hitbox.vertices.begin());

			// Create half-edge, setting main face to current face
			SATHalfEdge e = {v1, v2, (int)hitbox.faces.size(), -1};


			// If an opposing half-edge already exists, don't add this one
			for(int k = 0; k < hitbox.edges.size(); k++){

				const SATHalfEdge& e2 = hitbox.edges[k];

				if((e.v1 == e2.v1 && e.v2 == e2.v2) || (e.v1 == e2.v2 && e.v2 == e2.v1)){

					// Add existing edge index to face
					goto edgeLoop;
				}
			}


			// Find secondary face
			for(int k = 0; k < faces.size(); k++){

				const vector<Vec3>& f = faces[k];

				// Skip current face
				if(f == face)
					continue;

				const float threshold = 0.00001f;

				// If both vertices are present, then set this as the secondary face
				for(const Vec3& v1 : f){
					if(face[i].equalsWithinThreshold(v1, threshold)){
						for(const Vec3& v2 : f){
							if(face[j].equalsWithinThreshold(v2, threshold)){
								e.f2 = k;
								goto loopExit;
							}
						}
						break;
					}
				}
			}
		loopExit:;
			
			// No secondary face found
			if(e.f2 == -1)
				ntw::warning("Hitbox edge has no secondary face, is there a one-sided face or gap in the model?");

			// Add half-edge
			hitbox.edges.push_back(e);
			
		edgeLoop:;
		}
		
		// Create face
		SATFace satFace;
		satFace.position = {facePosition / (float)numVerts,};
		satFace.normal = crossProduct(face[0] - face[1], face[0] - face[2]).normalize();

		// Check that normal is facing outwards
		if(satFace.normal * satFace.position < 0)
			satFace.normal = -satFace.normal;


		// Add face
		hitbox.faces.push_back(satFace);
	}

	// Add edge indices to each face
	for(int i = 0; i < hitbox.edges.size(); i++){
		int f1 = hitbox.edges[i].f1;
		int f2 = hitbox.edges[i].f2;
		
		if(f1 != -1)	hitbox.faces[f1].edges.push_back(i);
		if(f2 != -1)	hitbox.faces[f2].edges.push_back(i);
	}

	// Add collider hitbox
	model->colliderHitboxes.push_back(hitbox);
}

void ntw::setModelProperties(Model* model){
	ntw::generateHitbox(model);
}


Model ntw::getPlane(){

	const vector<float> vertices = {
		-1,	1,	0,		-1,	-1,	0,		1,	1,	0,
		1,	-1,	0,		1,	1,	0,		-1,	-1,	0,
	};

	const vector<float> normals = {
		0, 0, 1,	0, 0, 1,	0, 0, 1,
		0, 0, 1,	0, 0, 1,	0, 0, 1,
	};

	const vector<float> texCoords = {
		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,
	};

	Model m;
	m.vertices = vertices;
	m.normals = normals;
	m.texCoords = texCoords;
	m.numVertices = (int)(vertices.size() / 3);

	return m;
}

Model ntw::getCube(bool smoothNormals){

	const vector<float> vertices = {
		-1,	-1,	1,		-1,	-1,	-1,		1,	-1,	1,
		1,	-1,	-1,		1,	-1,	1,		-1,	-1,	-1,

		1,	1,	1,		1,	1,	-1,		-1,	1,	1,
		-1,	1,	-1,		-1,	1,	1,		1,	1,	-1,

		-1,	1,	1,		-1,	1,	-1,		-1,	-1,	1,
		-1,	-1,	-1,		-1,	-1,	1,		-1,	1,	-1,

		1,	-1,	1,		1,	-1,	-1,		1,	1,	1,
		1,	1,	-1,		1,	1,	1,		1,	-1,	-1,

		-1,	1,	1,		-1,	-1,	1,		1,	1,	1,
		1,	-1,	1,		1,	1,	1,		-1,	-1,	1,

		-1,	-1,	-1,		-1,	1,	-1,		1,	-1,	-1,
		1,	1,	-1,		1,	-1,	-1,		-1,	1,	-1,
	};

	const vector<float> texCoords = {
		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,

		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,

		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,

		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,

		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,

		0, 0,	0, 1,	1, 0,
		1, 1,	1, 0,	0, 1,
	};

	Model m;
	m.vertices = vertices;
	m.normals = createNormals(vertices, smoothNormals);
	m.texCoords = texCoords;
	m.numVertices = (int)(vertices.size() / 3);

	return m;
}

Model ntw::getSphere(int segmentsU, int segmentsV, bool smoothNormals){

	vector<float> vertices;
	vector<float> texCoords;

	float angIncU = -360.0f / segmentsU;
	float angIncV = 180.0f / segmentsV;

	for(int u = 0; u < segmentsU; u++){
		for(int v = 0; v < segmentsV; v++){
			// Angle values
			float angU1 = u * angIncU;
			float angV1 = v * angIncV;
			float angU2 = angU1 + angIncU;
			float angV2 = angV1 + angIncV;

			// Generate vertex positions using the direction vector constructor
			Vec3 v1 = Vec3(-angU1, 90 + angV1);
			Vec3 v2 = Vec3(-angU2, 90 + angV1);
			Vec3 v3 = Vec3(-angU1, 90 + angV2);
			Vec3 v4 = Vec3(-angU2, 90 + angV2);

			// First and last segments are triangles
			if(v == 0){
				addVertex(v4);
				addVertex(v2);
				addVertex(v3);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV2 / 180);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV1 / 180);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV2 / 180);
			}
			else if(v == segmentsV - 1){
				addVertex(v1);
				addVertex(v3);
				addVertex(v2);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV1 / 180);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV2 / 180);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV1 / 180);
			}

			// Other segments are squares
			else{
				addVertex(v1);
				addVertex(v3);
				addVertex(v2);
				addVertex(v4);
				addVertex(v2);
				addVertex(v3);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV1 / 180);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV2 / 180);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV1 / 180);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV2 / 180);
				texCoords.push_back(-angU2 / 360);	texCoords.push_back(angV1 / 180);
				texCoords.push_back(-angU1 / 360);	texCoords.push_back(angV2 / 180);
			}
		}
	}


	Model m;
	m.vertices = vertices;
	m.normals = createNormals(vertices, smoothNormals);
	m.texCoords = texCoords;
	m.numVertices = (int)(vertices.size() / 3);

	return m;
}

Model ntw::getTransformedObjectModel(const Object& obj){

	Vec3 scale		= obj.getScale();
	Vec3 position	= obj.getPosition();

	Matrix rotation = Matrix(3, 3, true);
	rotation.rotate(obj.getRotation());

	Model* m = obj.getModel();
	vector<float>& vertices = m->vertices;
	vector<float>& normals = m->normals;
	vector<float> newVertices;
	vector<float> newNormals;

	for(int i = 0; i < m->numVertices; i++){

		// Place vertex into column vector to apply transformation
		Vec3 v = Vec3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
		Vec3 n = Vec3(normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]);

		// Scale
		v *= scale;

		// Rotate
		v = rotation * v;
		n = rotation * n;

		// Translate
		v += position;

		newVertices.push_back(v[0]);
		newVertices.push_back(v[1]);
		newVertices.push_back(v[2]);
		newNormals.push_back(n[0]);
		newNormals.push_back(n[1]);
		newNormals.push_back(n[2]);
	}

	Model newModel;
	newModel.vertices = newVertices;
	newModel.normals = newNormals;
	newModel.texCoords = m->texCoords;
	newModel.numVertices = m->numVertices;

	return newModel;
}

vector<float> ntw::createNormals(const vector<float>& vertices, bool smooth){
	
	vector<float> normals;

	// Store previous vertices for smoothing normals
	vector<Vec3> prevVertices;
	vector<vector<Vec3>> prevNormals;
	vector<vector<int>> prevNormalIndices;


	// Create normals for every face (triangle)
	for(int i = 0; i < vertices.size() / 9; i++){
		// Get face vertices
		Vec3 v1 = Vec3(vertices.at(i * 9),		vertices.at(i * 9 + 1), vertices.at(i * 9 + 2));
		Vec3 v2 = Vec3(vertices.at(i * 9 + 3),	vertices.at(i * 9 + 4), vertices.at(i * 9 + 5));
		Vec3 v3 = Vec3(vertices.at(i * 9 + 6),	vertices.at(i * 9 + 7), vertices.at(i * 9 + 8));

		// Cross product two vectors that define the face plane
		Vec3 n = -crossProduct(v1 - v2, v3 - v2);

		// Simply write normal for flat normals
		if(!smooth){
			// Write normal once per vertex
			for(int j = 0; j < 3; j++){
				normals.push_back(n[0]);
				normals.push_back(n[1]);
				normals.push_back(n[2]);
			}
		}

		// Smooth normals
		else{
			// Loop through each vertex in face
			for(int v = 0; v < 3; v++){

				// Get current vertex
				Vec3& currentVertex = v == 0 ? v1 : v == 1 ? v2 : v3;

				// If current vertex overlaps a previous one within a certain threshold
				int index = -1;
				for(auto pv = prevVertices.begin(); pv != prevVertices.end(); pv++){
					if(currentVertex.equalsWithinThreshold(*pv, 0.01f)){
						index = (int)(pv - prevVertices.begin());
						break;
					}
				}


				if(index != -1){
					// Store normal at this vertex if it is unique
					if(std::find(prevNormals[index].begin(), prevNormals[index].end(), n) != prevNormals[index].end())
						prevNormals[index].push_back(n);

					// Copy normal for this vertex
					Vec3 newNormal;

					// Sum normals for overlapping vertices
					for(auto pn = prevNormals[index].begin(); pn != prevNormals[index].end(); pn++)
						newNormal *= *pn;

					// Get previous normal
					//newNormal *= prevNormalIndices.size();

					// Add new normal to average
					//newNormal = (newNormal + n) / (prevNormalIndices.size() + 1);
					newNormal += n;

					// Write new normal
					normals.push_back(newNormal[0]);
					normals.push_back(newNormal[1]);
					normals.push_back(newNormal[2]);
					
					// Overwrite previous normals
					for(auto pn = prevNormalIndices[index].begin(); pn != prevNormalIndices[index].end(); pn++)
						for(int j = 0; j < 3; j++)
							normals[*pn + j] = newNormal[j];
				}

				// Write normal without modification
				else{
					normals.push_back(n[0]);
					normals.push_back(n[1]);
					normals.push_back(n[2]);

					// Keep track of vertices and counts
					index = (int)prevVertices.size();
					prevVertices.push_back(currentVertex);
					prevNormalIndices.push_back(vector<int>());

					// Store normal at this vertex
					prevNormals.push_back(vector<Vec3>());
					prevNormals[index].push_back(n);
				}

				prevNormalIndices[index].push_back(i * 9 + v * 3);
			}
		}
	}

	return normals;
}
