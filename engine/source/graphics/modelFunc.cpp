#include"modelFunc.h"

#include"math/mathFunc.h"
#include<unordered_map>

using std::unordered_map;

#define addVertex(v) vertices.push_back(v[0]); vertices.push_back(v[1]); vertices.push_back(v[2])


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
