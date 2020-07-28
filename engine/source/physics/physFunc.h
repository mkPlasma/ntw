#pragma once

#include"math/vec3.h"
#include"objects/collider.h"


namespace ntw{

	float getFaceToPointDistance(const SATFace& f, const Vec3& v);
	float getEdgeToEdgeDistance(const Hitbox& hitbox1, int edgeIndex1, const Hitbox& hitbox2, int edgeIndex2);


	vector<SATFace> getClippingPlanes(const Hitbox& hitbox, int faceIndex);
	vector<Vec3> clipFaces(const Hitbox& hitbox1, int faceIndex1, const Hitbox& hitbox2, int faceIndex2);


	float raycast(const Vec3& rayPosition, const Vec3& rayDirection, float maxDistance, const Hitbox& hitbox);
}
