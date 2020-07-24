#pragma once

/*
 *	physicsObject.h
 *
 *	Movable object with dynamic physics.
 *
 */

class PhysicsObject;

#include"object.h"
#include"math/matrix.h"


class PhysicsObject : public Object {
protected:
	const float mass_;
	float massInv_;

	bool useRotation_;
	bool useGravity_;
	bool useFriction_;

	// For simple dynamic objects only
	bool onGround_;
	bool onGroundClearNextFrame_;

	Matrix inertiaBase_;
	Matrix inertiaBaseInv_;
	Matrix tInertiaInv_;

	Vec3 tPosition_;
	Quaternion tRotation_;

	Vec3 velocity_;
	Vec3 angularVelocity_;

	Vec3 acceleration_;

public:
	PhysicsObject(World& world, Model* model, Material* material, HitboxType hitboxType, float mass);
	PhysicsObject(World& world, Model* model, Material* material, PhysicsType physicsType, HitboxType hitboxType, float mass);

	// Initialize inertia matrix and t-variables
	void initPhysics();

	// Update object physics
	void updatePhysics(float timeDelta);

	// Partial update during simulation step
	void tUpdatePhysics(float timeDelta);


	void updateTInertia();

	void setUseRotation(bool useRotation);
	void setUseGravity(bool useGravity);
	void setOnGround(bool onGround);


	void setVelocity(const Vec3& velocity);
	void addVelocity(const Vec3& velocity);
	void setAngularVelocity(const Vec3& angularVelocity);
	void addAngularVelocity(const Vec3& angularVelocity);

	void addAcceleration(const Vec3& acceleration);


	float getMass() const;
	float getMassInv() const;
	const Matrix& getInertiaBase() const;
	const Matrix& getTInertiaInv() const;

	bool useGravity() const;
	bool useRotation() const;
	bool onGround() const;

	const Vec3& getTPosition() const override;
	const Quaternion& getTRotation() const override;

	const Vec3& getVelocity() const;
	const Vec3& getAngularVelocity() const;
};
