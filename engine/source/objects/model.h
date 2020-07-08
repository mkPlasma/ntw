#pragma once

#include"math/vec3.h"
#include<vector>

using std::vector;


struct SATHalfEdge{
	// Vertex indices
	int v1;
	int v2;

	// Face indices
	// f2 is secondary face, -1 if no adjacent face
	int f1;
	int f2;
};


struct SATFace{
	// Position and normal of face plane
	Vec3 position;
	Vec3 normal;

	// Indices of edges that make up face (only vertices matter)
	vector<int> edges;
};


struct Hitbox{
	vector<Vec3> vertices;
	vector<SATHalfEdge> edges;
	vector<SATFace> faces;
};


struct Model{
	vector<float> vertices;
	vector<float> normals;
	vector<float> texCoords;
	int numVertices;

	vector<Vec3> hitbox;
	Hitbox hitboxSAT;
};
