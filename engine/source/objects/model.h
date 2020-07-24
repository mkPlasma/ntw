#pragma once

#include"hitbox.h"


struct Model{
	vector<float> vertices;
	vector<float> normals;
	vector<float> texCoords;
	int numVertices;

	Hitbox hitboxSAT;
	vector<Hitbox> colliderHitboxes;
};
