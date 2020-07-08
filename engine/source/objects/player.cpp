#include"player.h"

#include"physics/physDefine.h"
#include"objects/modelFunc.h"


Player::Player(ControlOptions& cOptions, Window& window) : PhysicsObject(nullptr, nullptr, PhysicsType::DYNAMIC_SIMPLE, HitboxType::MESH, 10),
	cOptions_(cOptions), window_(window), yaw_(0), pitch_(0), noclip_(false) {

}

void Player::initPlayer(){

	// Model (for hitbox)
	model_ = new Model(ntw::getCube());
	ntw::setModelProperties(model_);

	// Hitbox size
	setScale(Vec3(0.25f, 0.25f, 0.5f));

	// Disable rotation from physics updates
	setUseRotation(false);

	// Initialize look vectors
	updateLookVectors();
}

void Player::updatePlayer(float timeDelta){

	// Only update controls if mouse is locked
	if(window_.isMouseLocked()){

		// Mouse movement
		int mouseX = window_.getMouseX();
		int mouseY = window_.getMouseY();

		// Yaw (left-right)
		yaw_ -= (float)mouseX * cOptions_.mouseSensitivity;

		// Keep within 0-360 degree range
		while(yaw_ < 0)		yaw_ += 360;
		while(yaw_ > 360)	yaw_ -= 360;


		// Pitch (up-down)
		pitch_ -= (float)mouseY * cOptions_.mouseSensitivity;

		// Keep within -90-90 degree range
		if(pitch_ < -90)	pitch_ = -90;
		if(pitch_ > 90)		pitch_ = 90;

		// Update look vectors
		if(mouseX != 0 || mouseY != 0)
			updateLookVectors();

		// Keep mouse centered in window
		window_.centerMousePosition();


		// Noclip
		if(window_.isKeyPressed(NTW_KEY_NOCLIP)){
			noclip_ = !noclip_;
			setUseGravity(!noclip_);
			updateLookVectors();
		}

		// Movement
		float speed = 3;
		float acceleration = 30;

		Vec3 moveDir;

		// Get direction of movement
		if(window_.isKeyDown(NTW_KEY_FORWARDS))		moveDir += move_;
		if(window_.isKeyDown(NTW_KEY_BACKWARDS))	moveDir += -move_;
		if(window_.isKeyDown(NTW_KEY_RIGHT))		moveDir += lookRight_;
		if(window_.isKeyDown(NTW_KEY_LEFT))			moveDir += -lookRight_;

		// Disable friction while moving
		useFriction_ = moveDir.isZero();

		if(moveDir.nonzero()){
			// Normalize and multiply by acceleration
			moveDir.normalize();
			
			if(!noclip_){
				velocity_[0] += moveDir[0] * acceleration * timeDelta;
				velocity_[1] += moveDir[1] * acceleration * timeDelta;

				// Maximum speed
				float curSpeed = sqrtf((velocity_[0] * velocity_[0]) + (velocity_[1] * velocity_[1]));

				if(curSpeed > speed){
					velocity_[0] *= speed / curSpeed;
					velocity_[1] *= speed / curSpeed;
				}
			}
			else
				velocity_ = moveDir * speed;
		}
		else if(noclip_){
			velocity_.setX(0);
			velocity_.setY(0);
			velocity_.setZ(0);
		}


		// Jumping
		if(window_.isKeyDown(NTW_KEY_JUMP) && (onGround_ || noclip_))
			velocity_.setZ(noclip_ ? speed : 5);
	}

	/*
	// Average direction and multiply by speed
	moveDir *= speed / mc;

	if(mc != 0){
		// Subtract component of velocity on direction of movement
		// This limits speed increase when already moving in this direction
		//if(velocity_.nonzero())
		//	moveDir -= velocity_.compOn(moveDir);
		if(!noclip_)
			velocity_ += moveDir;
		else
			position_ += moveDir / 4.0f;
	}
	*/
}

void Player::updateLookVectors(){
	look_		= Vec3(yaw_ - 90, pitch_);
	lookRight_	= Vec3(yaw_ - 180, 0);
	lookUp_		= Vec3(yaw_ - 90, pitch_ + 90);
	move_		= noclip_ ? look_ : Vec3(yaw_ - 90, 0);
}


float Player::getYaw(){
	return yaw_;
}

float Player::getPitch(){
	return pitch_;
}

const Vec3& Player::getLookVector(){
	return look_;
}

const Vec3& Player::getLookRightVector(){
	return lookRight_;
}

const Vec3& Player::getLookUpVector(){
	return lookUp_;
}
