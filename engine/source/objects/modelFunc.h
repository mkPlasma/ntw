#pragma once

#include"objects/object.h"
#include"math/matrix.h"


namespace ntw{

	// Generate model hitbox from vertices
	void generateHitbox(Model* model);

	// Generate model properties (hitbox, etc.)
	void setModelProperties(Model* model);


	// Creates a model with texture coordinates
	Model getPlane();
	Model getCube(bool smoothNormals = false);
	Model getSphere(int segmentsU, int segmentsV, bool smoothNormals = false);

	// Applies object's transformations to its model for rendering
	Model getTransformedObjectModel(const Object& obj);

	// Creates normals given a set of vertices
	vector<float> createNormals(const vector<float>& vertices, bool smooth);
}
