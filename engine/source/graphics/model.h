#pragma once

/*
 *	model.h
 *
 *	Stores model geometry, texture coordinates, and material.
 *
 */

#include"texture.h"
#include<vector>

using std::vector;


struct Model{
	vector<float> vertices;
    vector<float> normals;
    vector<float> texCoords;
    int numVertices;
	Texture* texture;
};
