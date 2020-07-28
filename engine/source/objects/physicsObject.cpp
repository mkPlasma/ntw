#include"physicsObject.h"

#include"physics/physDefine.h"




PhysicsObject::PhysicsObject(World& world, Model* model, Material* material, float mass, PhysicsType physicsType) :
	Object(world, model, material, model == nullptr ? RenderType::NONE : RenderType::DYNAMIC, physicsType),
	mass_(mass), massInv_(1 / mass), useGravity_(true), gravityDirection_(Vec3(0, 0, -1)), useFriction_(true),
	onGround_(false), onGroundClearNextFrame_(false) {
	
}


void PhysicsObject::initPhysics(){

	// T-variables
	tPosition_ = position_;
	tRotation_ = rotation_;

	// Hitbox vertices to calculate inertia from
	vector<Vec3> vertices;

	// Get vertices from each collider
	for(const Collider& c : colliders_)
		std::copy(c.hitbox->vertices.begin(), c.hitbox->vertices.end(), std::back_inserter(vertices));


	// Point mass value
	float m = mass_ / vertices.size();

	// Base inertia tensor (no rotation)
	inertiaBase_ = Matrix(3, 3);

	// Set each tensor element
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){

			// Inertia sum
			float sum = 0;

			// Get inertia of each point
			for(Vec3 v : vertices){
				v *= scale_;
				sum += (i == j ? v.magnitude2() : 0) - (v[i] * v[j]);
			}

			inertiaBase_.set(i, j, sum * m);
		}
	}

	// Inverted inertia
	inertiaBaseInv_ = inertiaBase_.getInverse();

	updateTInertia();
}

void PhysicsObject::updatePhysics(){

	if(physicsType_ == PhysicsType::NONE || physicsType_ == PhysicsType::STATIC)
		return;

	// Keep track of previous position/rotation
	Vec3 prevPosition = position_;
	Quaternion prevRotation = rotation_;

	// Update velocity and position
	position_ += velocity_ * NTW_PHYS_TIME_DELTA;

	// Simple friction
	if(physicsType_ == PhysicsType::SIMPLE && useFriction_ && onGround_){
		float factor = 1 / powf(2.718f, 10 * NTW_PHYS_TIME_DELTA);
		velocity_ -= (velocity_ - velocity_.projOn(gravityDirection_)) * (1 - factor);
		angularVelocity_ *= factor;
	}

	// Update rotation
	float angVelMag = angularVelocity_.magnitude();
	rotation_.rotate(angularVelocity_, angVelMag * NTW_PHYS_TIME_DELTA);

	// Update t-variables
	tPosition_ = position_;
	tRotation_ = rotation_;

	// Clear on ground flag
	if(onGroundClearNextFrame_)
		onGround_ = false;

	onGroundClearNextFrame_ = onGround_;

	// If position/rotation have changed, cache hitbox again
	if(prevPosition != position_ || prevRotation != rotation_)
		hitboxCached_ = false;
}

void PhysicsObject::tUpdatePhysics(){

	if(physicsType_ == PhysicsType::NONE || physicsType_ == PhysicsType::STATIC)
		return;

	// Keep track of previous position/rotation
	Vec3 prevTPosition = tPosition_;
	Quaternion prevTRotation = tRotation_;

	// Update velocity and position
	tPosition_ = position_ + (velocity_) *NTW_PHYS_TIME_DELTA;

	// Update rotation
	float angVelMag = angularVelocity_.magnitude();
	tRotation_ = rotation_;
	tRotation_.rotate(angularVelocity_, angVelMag * NTW_PHYS_TIME_DELTA);

	if(angVelMag != 0)
		updateTInertia();

	// If position/rotation have changed, cache hitbox again
	if(prevTPosition != tPosition_ || prevTRotation != tRotation_)
		hitboxCached_ = false;
}


void PhysicsObject::updateTInertia(){
	Matrix tRot = Matrix(3, 3, true).rotate(tRotation_);
	tInertiaInv_ = tRot * inertiaBaseInv_ * tRot.getTranspose();
}

void PhysicsObject::setUseGravity(bool useGravity){
	useGravity_ = useGravity;
}

void PhysicsObject::setGravityDirection(const Vec3& gravityDirection){
	gravityDirection_ = gravityDirection;
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

bool PhysicsObject::useGravity() const{
	return useGravity_;
}

const Vec3& PhysicsObject::getGravityDirection() const{
	return gravityDirection_;
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
