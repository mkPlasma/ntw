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
	float mass_;
	float massInv_;

	bool allowRotation_;

	Matrix inertiaBase_;
	Matrix inertiaBaseInv_;
	Matrix tInertiaInv_;


	Vec3 tPosition_;
	Quaternion tRotation_;


	Vec3 velocity_;
	Vec3 angularVelocity_;

	Vec3 acceleration_;

public:
	PhysicsObject(Model* model, const int& hitboxType, const float& mass);

	// Initialize inertia matrix and t-variables
	void init();

	// Update object physics
	void update();

	// Partial update during simulation step
	void tUpdate();


	void updateTInertia();

	void setAllowRotation(bool allowRotation);

	void setVelocity(const Vec3& velocity);
	void addVelocity(const Vec3& velocity);
	void setAngularVelocity(const Vec3& angularVelocity);
	void addAngularVelocity(const Vec3& angularVelocity);

	void addAcceleration(const Vec3& acceleration);


	float getMass();
	float getMassInv();
	Matrix getInertiaBase();
	Matrix getTInertiaInv();

	Vec3 getTPosition();
	Quaternion getTRotation();

	Vec3 getVelocity();
	Vec3 getAngularVelocity();
};
