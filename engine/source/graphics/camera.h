#pragma once

#include"math/vec3.h"
#include"math/matrix.h"


struct Camera{
	Vec3 position;
	Vec3 velocity;
	float yaw;
	float pitch;
	float roll;
	Matrix rotationMatrix;
};
