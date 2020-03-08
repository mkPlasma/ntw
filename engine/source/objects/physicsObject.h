#pragma once

/*
 *	physicsObject.h
 *
 *	Movable object with dynamic physics.
 *
 */

#include"object.h"
#include"math/matrix.h"


class PhysicsObject : public Object {
protected:
	const float mass_;
	float massInv_;

	bool useRotation_;
	bool useGravity_;

	Matrix inertiaBase_;
	Matrix inertiaBaseInv_;
	Matrix tInertiaInv_;


	Vec3 tPosition_;
	Quaternion tRotation_;


	Vec3 velocity_;
	Vec3 angularVelocity_;

	Vec3 acceleration_;

public:
	PhysicsObject(Model* model, Material* material, HitboxType hitboxType, float mass);

	// Initialize inertia matrix and t-variables
	void initPhysics();

	// Update object physics
	void updatePhysics();

	// Partial update during simulation step
	void tUpdatePhysics();


	void updateTInertia();

	void setUseRotation(bool useRotation);
	void setUseGravity(bool useGravity);

	void setVelocity(const Vec3& velocity);
	void addVelocity(const Vec3& velocity);
	void setAngularVelocity(const Vec3& angularVelocity);
	void addAngularVelocity(const Vec3& angularVelocity);

	void addAcceleration(const Vec3& acceleration);


	float getMass();
	float getMassInv();
	const Matrix& getInertiaBase();
	const Matrix& getTInertiaInv();

	bool useGravity();
	bool useRotation();

	const Vec3& getTPosition();
	const Quaternion& getTRotation();

	const Vec3& getVelocity();
	const Vec3& getAngularVelocity();
};
