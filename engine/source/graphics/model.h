#pragma once

#include<vector>

using std::vector;


struct Model{
	vector<float> vertices;
    vector<float> normals;
    vector<float> texCoords;
    int numVertices;
};
