#pragma once

/*
 *	constraint.h
 *
 *	Physics object constraint.
 *
 */

#include"objects/physicsObject.h"
#include"physics/physStruct.h"
#include<vector>

using std::vector;


class Constraint{
protected:

	Object* object1_;
	Object* object2_;

	// Individual mass values to speed up mass matrix multiplication
	float invMass1_;
	float invMass2_;
	Matrix invInertia1_;
	Matrix invInertia2_;

	// Velocity vector
	Matrix vel_;

	// Jacobian
	Matrix jac_;

	// Constraint value
	float constraint_;

	// Inverse mass times jacobian
	Matrix invMassJt_;
	
	// Lagrangian multiplied
	float lambda_;

	// Corrective velocity
	Matrix velCor_;

	float bias_;

	bool firstSolve_;

	// If true, constraint will be applied on C < 0 rather than C != 0
	bool constrainGreaterThanZero_;

public:
	Constraint(Object* object1, Object* object2, bool constrainGreaterThanZero = false);

	virtual void init();
	virtual bool solve();

	// Calculate constraint values
	// Return true if constraint is satisfied
	virtual inline bool calcConstraint();
	virtual inline void calcInvMassJt();
	virtual inline void calcLambda();
	virtual inline void calcVelCor();

	// Apply corrective velocities
	void apply();

	ObjectPair getObjects();
};
