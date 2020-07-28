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

	bool useGravity_;
	Vec3 gravityDirection_;

	bool useFriction_;

	// For simple dynamic objects only
	bool onGround_;
	bool onGroundClearNextFrame_;

	Matrix inertiaBase_;
	Matrix inertiaBaseInv_;
	Matrix tInertiaInv_;

	Vec3 velocity_;
	Vec3 angularVelocity_;

	Vec3 tPosition_;
	Quaternion tRotation_;

public:
	PhysicsObject(World& world, Model* model, Material* material, float mass, PhysicsType physicsType = PhysicsType::RIGID_BODY);

	// Initialize inertia matrix and t-variables
	void initPhysics();

	// Update object physics
	void updatePhysics();

	// Partial update during simulation step
	void tUpdatePhysics();


	void updateTInertia();

	void setUseGravity(bool useGravity);
	void setGravityDirection(const Vec3& gravityDirection);

	void setOnGround(bool onGround);


	void setVelocity(const Vec3& velocity);
	void addVelocity(const Vec3& velocity);
	void setAngularVelocity(const Vec3& angularVelocity);
	void addAngularVelocity(const Vec3& angularVelocity);


	float getMass() const;
	float getMassInv() const;
	const Matrix& getInertiaBase() const;
	const Matrix& getTInertiaInv() const;

	bool useGravity() const;
	const Vec3& getGravityDirection() const;

	bool onGround() const;

	const Vec3& getTPosition() const override;
	const Quaternion& getTRotation() const override;

	const Vec3& getVelocity() const;
	const Vec3& getAngularVelocity() const;
};
