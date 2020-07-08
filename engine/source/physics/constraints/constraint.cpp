#include"constraint.h"

#include"physics/physDefine.h"


Constraint::Constraint(Object* object1, Object* object2, bool constrainGreaterThanZero)
	: object1_(object1), object2_(object2), bias_(0), firstSolve_(true), constrainGreaterThanZero_(constrainGreaterThanZero) {
	
}

void Constraint::init(){

	bool obj1Dynamic	= object1_->getPhysicsType() == PhysicsType::DYNAMIC;
	bool obj2Dynamic	= object2_->getPhysicsType() == PhysicsType::DYNAMIC;
	bool obj1Phys		= obj1Dynamic || object1_->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;
	bool obj2Phys		= obj2Dynamic || object2_->getPhysicsType() == PhysicsType::DYNAMIC_SIMPLE;

	PhysicsObject* pObj1 = obj1Phys ? (PhysicsObject*)object1_ : nullptr;
	PhysicsObject* pObj2 = obj2Phys ? (PhysicsObject*)object2_ : nullptr;

	if(firstSolve_){
		// Initialize jacobian, it will be set by the specific constraint
		jac_ = Matrix(1, 12);

		vel_ = Matrix(12, 1);

		// Inverted mass matrix (use zero matrix for immovable static and semi-dynamic objects)
		// Individual mass values to speed up mass matrix multiplication
		invMass1_ = obj1Dynamic ? pObj1->getMassInv() : 0;
		invMass2_ = obj2Dynamic ? pObj2->getMassInv() : 0;
	}

	// Velocity vector
	vel_.place(0, 0, obj1Phys ? pObj1->getVelocity()		* NTW_PHYS_TIME_DELTA : Vec3());
	vel_.place(3, 0, obj1Phys ? pObj1->getAngularVelocity()	* NTW_PHYS_TIME_DELTA : Vec3());
	vel_.place(6, 0, obj2Phys ? pObj2->getVelocity()		* NTW_PHYS_TIME_DELTA : Vec3());
	vel_.place(9, 0, obj2Phys ? pObj2->getAngularVelocity()	* NTW_PHYS_TIME_DELTA : Vec3());

	invInertia1_ = obj1Dynamic ? pObj1->getTInertiaInv() : Matrix(3, 3);
	invInertia2_ = obj2Dynamic ? pObj2->getTInertiaInv() : Matrix(3, 3);
}

void Constraint::solve(){

	// Calculate constraint value and check if it should be enforced
	if(calcConstraint())
		return;

	// Solve
	calcInvMassJt();
	calcLambda();
	calcVelCor();
	apply();
}

bool Constraint::calcConstraint(){
	// Calculate
	constraint_ = (jac_ * vel_).get(0, 0) + bias_;

	// Return true if constraint is satisfied
	return abs(constraint_) < PHYS_CONSTRAINT_THRESHOLD || (constrainGreaterThanZero_ && constraint_ > 0);
}

void Constraint::calcInvMassJt(){

	// Get inverse mass matrix times jacobian transpose
	invMassJt_ = Matrix(12, 1);
	invMassJt_.place(0, 0, Vec3(jac_.get(0, 0) * invMass1_, jac_.get(0, 1) * invMass1_, jac_.get(0, 2) * invMass1_));
	invMassJt_.place(3, 0, invInertia1_ * Vec3(jac_.get(0, 3), jac_.get(0, 4), jac_.get(0, 5)));
	invMassJt_.place(6, 0, Vec3(jac_.get(0, 6) * invMass2_, jac_.get(0, 7) * invMass2_, jac_.get(0, 8) * invMass2_));
	invMassJt_.place(9, 0, invInertia2_ * Vec3(jac_.get(0, 9), jac_.get(0, 10), jac_.get(0, 11)));

	/*
	Matrix tmp = Matrix(12, 12);
	tmp.set(0, 0, invMass1_);
	tmp.set(1, 1, invMass1_);
	tmp.set(2, 2, invMass1_);
	tmp.place(3, 3, invInertia1_);
	tmp.set(4, 4, invMass2_);
	tmp.set(5, 5, invMass2_);
	tmp.set(6, 6, invMass2_);
	tmp.place(9, 9, invInertia2_);

	Matrix tmp2 = tmp * jac_.getTranspose();
	//invMassJt_ = tmp2;
	*/
}

void Constraint::calcLambda(){
	lambda_ = -constraint_ / ((jac_ * invMassJt_ * NTW_PHYS_TIME_DELTA).get(0, 0));
}

void Constraint::calcVelCor(){
	velCor_ = invMassJt_ * lambda_;
}

void Constraint::apply(){

	vel_ += velCor_;

	// Apply corrective velocities if object is dymamic
	if(object1_->getPhysicsType() == PhysicsType::DYNAMIC){
		PhysicsObject* obj = (PhysicsObject*)object1_;
		obj->addVelocity(Vec3(velCor_.get(0, 0), velCor_.get(1, 0), velCor_.get(2, 0)));
		obj->addAngularVelocity(Vec3(velCor_.get(3, 0), velCor_.get(4, 0), velCor_.get(5, 0)));
	}
	if(object2_->getPhysicsType() == PhysicsType::DYNAMIC){
		PhysicsObject* obj = (PhysicsObject*)object2_;
		obj->addVelocity(Vec3(velCor_.get(6, 0), velCor_.get(7, 0), velCor_.get(8, 0)));
		obj->addAngularVelocity(Vec3(velCor_.get(9, 0), velCor_.get(10, 0), velCor_.get(11, 0)));
	}
}

bool Constraint::isSolved(){
	return calcConstraint();
}

ObjectPair Constraint::getObjects(){
	return {object1_, object2_};
}
