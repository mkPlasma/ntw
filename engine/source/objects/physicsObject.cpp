#include"physicsObject.h"

#include"physics/physDefine.h"


PhysicsObject::PhysicsObject(Model* model, const int& hitboxType, const float& mass) :
	Object(model, model == nullptr ? RENDER_NONE : RENDER_DYNAMIC, PHYS_DYNAMIC, hitboxType),
	mass_(mass), massInv_(1 / mass), allowRotation_(true) {
	
}

void PhysicsObject::init(){

	// T-variables
	tPosition_ = position_;
	tRotation_ = rotation_;

	// Calculate inertia values if rotation is allowed
	if(allowRotation_){
		// Base inertia tensor (no rotation)
		inertiaBase_ = Matrix(3, 3);

		// Cube
		float sx2 = powf(scale_[0] * 2, 2);
		float sy2 = powf(scale_[1] * 2, 2);
		float sz2 = powf(scale_[2] * 2, 2);
		inertiaBase_.set(0, 0, (mass_ * (sy2 + sz2)) / 12);
		inertiaBase_.set(1, 1, (mass_ * (sx2 + sz2)) / 12);
		inertiaBase_.set(2, 2, (mass_ * (sx2 + sy2)) / 12);

		// Inverted inertia
		inertiaBaseInv_ = inertiaBase_.getInverse();

		updateTInertia();
	}
	// If not, set inverse inertia to all zeroes (infinite inertia to disable rotation)
	else
		tInertiaInv_ = Matrix(3, 3);
}

void PhysicsObject::update(){

	// Update velocity and position
	velocity_ += acceleration_ * PHYS_TIMESTEP;
	position_ += velocity_ * PHYS_TIMESTEP;

	// Update rotation
	if(allowRotation_){
		float angVelMag = angularVelocity_.magnitude();
		rotation_.rotate(angularVelocity_, angVelMag * PHYS_TIMESTEP);
	}

	// Clear accelerations
	acceleration_ = Vec3();

	// Update t-variables
	tPosition_ = position_;
	tRotation_ = rotation_;
}

void PhysicsObject::tUpdate(){

	// Update position
	tPosition_ += (velocity_ + acceleration_) * PHYS_TIMESTEP;

	// Update rotation
	if(allowRotation_){
		float angVelMag = angularVelocity_.magnitude();
		tRotation_.rotate(angularVelocity_, angVelMag * PHYS_TIMESTEP);

		if(angVelMag != 0)
			updateTInertia();
	}
}


void PhysicsObject::updateTInertia(){
	Matrix tRot = Matrix(3, 3, true).rotate(tRotation_);
	tInertiaInv_ = tRot * inertiaBaseInv_ * tRot.getTranspose();
}

void PhysicsObject::setAllowRotation(bool allowRotation){
	allowRotation_ = allowRotation;
}

void PhysicsObject::setVelocity(const Vec3& velocity){
	velocity_ = velocity;
}

void PhysicsObject::addVelocity(const Vec3& velocity){
	velocity_ += velocity;
}

void PhysicsObject::setAngularVelocity(const Vec3& angularVelocity){
	angularVelocity_ = angularVelocity;
}

void PhysicsObject::addAngularVelocity(const Vec3& angularVelocity){
	angularVelocity_ += angularVelocity;
}

void PhysicsObject::addAcceleration(const Vec3& acceleration){
	acceleration_ += acceleration;
}

float PhysicsObject::getMass(){
	return mass_;
}

float PhysicsObject::getMassInv(){
	return massInv_;
}

Matrix PhysicsObject::getInertiaBase(){
	return inertiaBase_;
}

Matrix PhysicsObject::getTInertiaInv(){
	return tInertiaInv_;
}

Vec3 PhysicsObject::getTPosition(){
	return tPosition_;
}

Quaternion PhysicsObject::getTRotation(){
	return tRotation_;
}

Vec3 PhysicsObject::getVelocity(){
	return velocity_;
}

Vec3 PhysicsObject::getAngularVelocity(){
	return angularVelocity_;
}
