#include"player.h"

#include"physics/physDefine.h"
#include"objects/modelFunc.h"
#include"core/world.h"
#include"math/mathFunc.h"
#include<limits>
#include<math.h>


Player::Player(World& world, Model* model, ControlOptions& cOptions, Window& window) : PhysicsObject(world, model, nullptr, 10, PhysicsType::SIMPLE),
	cOptions_(cOptions), window_(window), yaw_(0), pitch_(0), noclip_(false) {

	setRenderType(RenderType::NONE);

	// Hitbox size
	setScale(Vec3(0.25f, 0.25f, 0.5f));

	// Initialize look vectors
	updateLookVectors();
}

void Player::updatePlayer(float timeDelta, bool updatePhysics){

	// Update mouse movement only when mouse locked
	if(window_.isMouseLocked()){

		// Mouse movement
		int mouseX = window_.getMouseX();
		int mouseY = window_.getMouseY();

		// Yaw (left-right)
		yawDifference_ = -(float)mouseX * cOptions_.mouseSensitivity;
		yaw_ += yawDifference_;

		// Keep within 0-360 degree range
		while(yaw_ < 0)		yaw_ += 360;
		while(yaw_ > 360)	yaw_ -= 360;


		// Pitch (up-down)
		float oldPitch = pitch_;
		pitch_ -= (float)mouseY * cOptions_.mouseSensitivity;

		// Keep within -89-89 degree range
		if(pitch_ < -89)	pitch_ = -89;
		if(pitch_ > 89)		pitch_ = 89;

		pitchDifference_ = pitch_ - oldPitch;


		// Update look vectors
		if(mouseX != 0 || mouseY != 0)
			updateLookVectors();

		// Keep mouse centered in window
		window_.centerMousePosition();
	}
	else{
		yawDifference_ = 0;
		pitchDifference_ = 0;
	}


	// Noclip
	if(window_.isKeyPressed(NTW_KEY_NOCLIP)){
		noclip_ = !noclip_;
		setUseGravity(!noclip_);
		updateLookVectors();
	}


	// Update movement only during physics update
	if(!updatePhysics){

		// Movement
		float maxSpeed = 3;
		float acceleration = 30;

		Vec3 moveDir;

		// Get direction of movement
		if(window_.isKeyDown(NTW_KEY_FORWARDS))		moveDir += move_;
		if(window_.isKeyDown(NTW_KEY_BACKWARDS))	moveDir -= move_;
		if(window_.isKeyDown(NTW_KEY_RIGHT))		moveDir += lookRight_;
		if(window_.isKeyDown(NTW_KEY_LEFT))			moveDir -= lookRight_;


		// Disable friction while moving
		useFriction_ = moveDir.isZero();

		if(moveDir.nonzero()){
			// Normalize and multiply by acceleration
			moveDir.normalize();

			if(!noclip_){
				// Add acceleration
				velocity_ += moveDir * acceleration * NTW_PHYS_TIME_DELTA;

				// Current vertical velocity
				Vec3 verticalVelocity = velocity_.projOn(gravityDirection_);

				// Current horizontal speed
				float speed = (velocity_ - verticalVelocity).magnitude();

				// Adjust maximum speed based on collisions
				//for(const ContactInfo& c : contacts_)
				//	if(c.object->getPhysicsType() == PhysicsType::STATIC)
				//		moveDir -= moveDir.clampedProjOn(portalRotation_ * -c.normal);

				maxSpeed *= moveDir.magnitude();

				if(speed > maxSpeed){
					// Remove vertical component and apply speed correction, then re-add vertical component
					velocity_ -= verticalVelocity;
					velocity_ *= maxSpeed / speed;
					velocity_ += verticalVelocity;
				}
			}
			else
				velocity_ = moveDir * maxSpeed;
		}
		else if(noclip_){
			velocity_.setX(0);
			velocity_.setY(0);
			velocity_.setZ(0);
		}


		// Jumping
		if(window_.isKeyDown(NTW_KEY_JUMP) && (onGround_ || noclip_)){

			// Set vertical velocity to jump speed
			Vec3 verticalVelocity = velocity_.projOn(gravityDirection_);
			velocity_ += (-gravityDirection_ * (noclip_ ? maxSpeed : 5)) - verticalVelocity;
		}

		updateEyePosition();
	}

	// Object grabbing
	if(window_.isKeyPressed(NTW_KEY_GRAB)){

		// Release held object
		if(heldObject_ != nullptr){

			// Restore original properties
			heldObject_->setPhysicsType(heldObjectPhysicsType_);
			heldObject_->setUseGravity(heldObjectUseGravity_);

			heldObject_ = nullptr;
		}
		else{
			const float maxDistance = 5;
			const float maxMass = 10;

			// Cast ray and get closest object
			vector<Object*> objects = world_.getPhysicsEngine().castRay(eyePosition_, look_, maxDistance);

			Object* object = nullptr;
			float minDistance = std::numeric_limits<float>::max();

			for(int i = 0; i < objects.size(); i++){

				float distance = (position_ - objects[i]->getPosition()).magnitude2();

				if(distance < minDistance && objects[i] != this){
					object = objects[i];
					minDistance = distance;
				}
			}

			// Check that object is valid for holding
			if(object && (object->getPhysicsType() == PhysicsType::RIGID_BODY || object->getPhysicsType() == PhysicsType::SIMPLE) &&
				((PhysicsObject*)object)->getMass() <= maxMass){

				// Store held object
				heldObject_ = (PhysicsObject*)object;

				// Store original physics type and set to dynamic simple
				heldObjectPhysicsType_ = heldObject_->getPhysicsType();
				heldObject_->setPhysicsType(PhysicsType::SIMPLE);

				// Disable gravity while held
				heldObjectUseGravity_ = heldObject_->useGravity();
				heldObject_->setUseGravity(false);
			}
		}
	}

	// Update held object
	if(updatePhysics && heldObject_ != nullptr){
		
		const float holdDistance = 1;

		// Update position
		Vec3 newPosition = eyePosition_ + (look_ * holdDistance);
		heldObject_->setVelocity((newPosition - heldObject_->getPosition()) / NTW_PHYS_TIME_DELTA);

		// Rotation
		heldObject_->setAngularVelocity(Vec3(0, 0, ntw::toRadians(yawDifference_) / NTW_PHYS_TIME_DELTA));
	}
}

void Player::updateEyePosition(){
	eyePosition_ = position_;
	eyePosition_[2] = position_[2] + (NTW_PLAYER_EYE_LEVEL - 0.5f) * scale_[2];
}

void Player::updateLookVectors(){
	look_		= portalRotation_ * Vec3(yaw_, pitch_);
	lookRight_	= portalRotation_ * Vec3(yaw_ - 90, 0);
	lookUp_		= portalRotation_ * Vec3(yaw_, pitch_ + 90);
	move_		= noclip_ ? look_ : portalRotation_ * Vec3(yaw_, 0);
}


void Player::setYaw(float yaw){
	yaw_ = yaw;
}

void Player::setPitch(float pitch){
	pitch_ = pitch;
}

void Player::addPortalRotation(const Matrix& portalRotation){
	portalRotation_ = portalRotation * portalRotation_;

	rotation_ = Quaternion(portalRotation_);
	updateLookVectors();
}


bool Player::isPlayer() const{
	return true;
}


float Player::getYaw() const{
	return yaw_;
}

float Player::getPitch() const{
	return pitch_;
}

const Vec3& Player::getEyePosition() const{
	return eyePosition_;
}

const Vec3& Player::getLookVector() const{
	return look_;
}

const Vec3& Player::getLookRightVector() const{
	return lookRight_;
}

const Vec3& Player::getLookUpVector() const{
	return lookUp_;
}

const Matrix& Player::getPortalRotation() const{
	return portalRotation_;
}
