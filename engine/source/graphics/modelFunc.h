#pragma once

#include"objects/object.h"
#include"math/matrix.h"


namespace ntw{

	// Creates a model with texture coordinates
	Model getPlane();
	Model getCube(const bool& smoothNormals = false);
	Model getSphere(const int& segmentsU, const int& segmentsV, const bool& smoothNormals = false);

	// Applies object's transformations to its model for rendering
	Model getTransformedObjectModel(const Object& obj);

	// Creates normals given a set of vertices
	vector<float> createNormals(const vector<float>& vertices, const bool& smooth);
}
