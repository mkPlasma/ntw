#include"physicsObject.h"

#include"physics/physDefine.h"




PhysicsObject::PhysicsObject(Model* model, Material* material, HitboxType hitboxType, float mass) :
	PhysicsObject(model, material, PhysicsType::DYNAMIC, hitboxType, mass) {
}

PhysicsObject::PhysicsObject(Model* model, Material* material, PhysicsType physicsType, HitboxType hitboxType, float mass) :
	Object(model, material, model == nullptr ? RenderType::NONE : RenderType::DYNAMIC, physicsType, hitboxType),
	mass_(mass), massInv_(1 / mass), useRotation_(true), useGravity_(true), useFriction_(true),
	onGround_(false), onGroundClearNextFrame_(false) {
	
}


void PhysicsObject::initPhysics(){

	// T-variables
	tPosition_ = position_;
	tRotation_ = rotation_;

	// Calculate inertia values if rotation is allowed
	if(useRotation_){

		// Hitbox vertices to calculate inertia from
		const vector<Vec3>& verts = model_->hitboxSAT.vertices;

		// Point mass value
		float m = mass_ / verts.size();

		// Base inertia tensor (no rotation)
		inertiaBase_ = Matrix(3, 3);

		// Set each tensor element
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){

				// Inertia sum
				float sum = 0;

				// Get inertia of each point
				for(Vec3 v : verts){
					v *= scale_;
					sum += (i == j ? v.magnitude2() : 0) - (v[i] * v[j]);
				}

				inertiaBase_.set(i, j, sum * m);
			}
		}

		// Cube
		/*
		float sx2 = powf(scale_[0] * 2, 2);
		float sy2 = powf(scale_[1] * 2, 2);
		float sz2 = powf(scale_[2] * 2, 2);
		inertiaBase_.set(0, 0, (mass_ * (sy2 + sz2)) / 12);
		inertiaBase_.set(1, 1, (mass_ * (sx2 + sz2)) / 12);
		inertiaBase_.set(2, 2, (mass_ * (sx2 + sy2)) / 12);
		*/

		// Inverted inertia
		inertiaBaseInv_ = inertiaBase_.getInverse();

		updateTInertia();
	}
	// If not, set inverse inertia to all zeroes (infinite inertia to disable rotation)
	else
		tInertiaInv_ = Matrix(3, 3);
}

void PhysicsObject::updatePhysics(float timeDelta){

	if(physicsType_ == PhysicsType::NONE || physicsType_ == PhysicsType::STATIC)
		return;

	// Rigid body time delta
	if(physicsType_ == PhysicsType::DYNAMIC)
		timeDelta = NTW_PHYS_TIME_DELTA;

	// Update velocity and position
	//velocity_ += acceleration_;
	position_ += velocity_ * timeDelta;

	// Simple friction
	if(physicsType_ == PhysicsType::DYNAMIC_SIMPLE && useFriction_ && onGround_){
		velocity_[0] /= (powf(2.718f, 10 * timeDelta));
		velocity_[1] /= (powf(2.718f, 10 * timeDelta));
	}

	// Update rotation
	if(useRotation_){
		float angVelMag = angularVelocity_.magnitude();
		rotation_.rotate(angularVelocity_, angVelMag * timeDelta);
	}

	// Clear accelerations
	//acceleration_ = Vec3();

	// Update t-variables
	tPosition_ = position_;
	tRotation_ = rotation_;

	// Clear on ground flag
	if(onGroundClearNextFrame_)
		onGround_ = false;

	onGroundClearNextFrame_ = onGround_;

	/*
	// Update velocity and position
	velocity_ += acceleration_ * PHYS_TIMESTEP;
	position_ += velocity_ * PHYS_TIMESTEP;

	// Update rotation
	if(useRotation_){
		float angVelMag = angularVelocity_.magnitude();
		rotation_.rotate(angularVelocity_, angVelMag * PHYS_TIMESTEP);
	}

	// Clear accelerations
	acceleration_ = Vec3();

	// Update t-variables
	tPosition_ = position_;
	tRotation_ = rotation_;
	*/
}

void PhysicsObject::tUpdatePhysics(float timeDelta){

	if(physicsType_ == PhysicsType::NONE || physicsType_ == PhysicsType::STATIC)
		return;

	// Rigid body time delta
	if(physicsType_ == PhysicsType::DYNAMIC)
		timeDelta = NTW_PHYS_TIME_DELTA;

	// Update velocity and position
	tPosition_ = position_ + (velocity_) * timeDelta;

	// Update rotation
	if(useRotation_){
		float angVelMag = angularVelocity_.magnitude();
		tRotation_ = rotation_;
		tRotation_.rotate(angularVelocity_, angVelMag * timeDelta);

		if(angVelMag != 0)
			updateTInertia();
	}

	/*
	// Update position
	tPosition_ += (velocity_ + acceleration_) * PHYS_TIMESTEP;

	// Update rotation
	if(useRotation_){
		float angVelMag = angularVelocity_.magnitude();
		tRotation_.rotate(angularVelocity_, angVelMag * PHYS_TIMESTEP);

		if(angVelMag != 0)
			updateTInertia();
	}
	*/
}


void PhysicsObject::updateTInertia(){
	Matrix tRot = Matrix(3, 3, true).rotate(tRotation_);
	tInertiaInv_ = tRot * inertiaBaseInv_ * tRot.getTranspose();
}

void PhysicsObject::setUseRotation(bool useRotation){
	useRotation_ = useRotation;
}

void PhysicsObject::setUseGravity(bool useGravity){
	useGravity_ = useGravity;
}

void PhysicsObject::setOnGround(bool onGround){

	// Keep flag from being cleared while object is still on ground
	if(onGround_)
		onGroundClearNextFrame_ = false;

	onGround_ = onGround;
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

float PhysicsObject::getMass() const{
	return mass_;
}

float PhysicsObject::getMassInv() const{
	return massInv_;
}

const Matrix& PhysicsObject::getInertiaBase() const{
	return inertiaBase_;
}

const Matrix& PhysicsObject::getTInertiaInv() const{
	return tInertiaInv_;
}

bool PhysicsObject::useRotation() const{
	return useRotation_;
}

bool PhysicsObject::useGravity() const{
	return useGravity_;
}

bool PhysicsObject::onGround() const{
	return onGround_;
}

const Vec3& PhysicsObject::getTPosition() const{
	return tPosition_;
}

const Quaternion& PhysicsObject::getTRotation() const{
	return tRotation_;
}

const Vec3& PhysicsObject::getVelocity() const{
	return velocity_;
}

const Vec3& PhysicsObject::getAngularVelocity() const{
	return angularVelocity_;
}
