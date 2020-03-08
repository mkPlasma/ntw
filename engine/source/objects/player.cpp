#include"player.h"

#include"physics/physDefine.h"


Player::Player(ControlOptions& cOptions, Window& window) : PhysicsObject(nullptr, nullptr, HitboxType::CUBE, 5),
	cOptions_(cOptions), window_(window), yaw_(0), pitch_(0), noclip_(false) {

}

void Player::initPlayer(){

	// Hitbox size
	setScale(Vec3(0.25f, 0.25f, 1.0f));

	// Disable rotation from physics updates
	setUseRotation(false);

	// Initialize look vectors
	updateLookVectors();

	// TEMPORARY NOCLIP ENABLE
	/*
	noclip_ = true;
	hitboxType_ = NONE;
	setUseGravity(false);
	*/
}

void Player::updatePlayer(bool updatePhysics){

	// Only update if mouse is locked
	if(!window_.isMouseLocked())
		return;


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


	// Noclip
	//if(window_.isKeyDown(cOptions_.keys[NTW_KEY_NOCLIP]))

	// Update look vectors
	if(mouseX != 0 || mouseY != 0)
		updateLookVectors();

	// Keep mouse centered in window
	window_.centerMousePosition();


	// Movement
	float speedMult = 0.2f;
	Vec3 velAdd;
	int mc = 0;
	
	// Get direction of movement
	if(window_.isKeyDown(cOptions_.keys[NTW_KEY_FORWARDS])){	velAdd += move_;		mc++;}
	if(window_.isKeyDown(cOptions_.keys[NTW_KEY_BACKWARDS])){	velAdd += -move_;		mc++;}
	if(window_.isKeyDown(cOptions_.keys[NTW_KEY_RIGHT])){		velAdd += lookRight_;	mc++;}
	if(window_.isKeyDown(cOptions_.keys[NTW_KEY_LEFT])){		velAdd += -lookRight_;	mc++;}

	// Average direction and multiply by speed
	velAdd *= speedMult / mc;

	if(mc != 0){
		// Subtract component of velocity on direction of movement
		// This limits speed increase when already moving in this direction
		//if(velocity_.nonzero())
		//	velAdd -= velocity_.compOn(velAdd);
		if(!noclip_)
			velocity_ += velAdd;
		else
			position_ += velAdd / 4.0f;
	}

	// Jumping
	if(window_.isKeyDown(cOptions_.keys[NTW_KEY_JUMP]) && !noclip_)
		velocity_.setZ(2.0f);



	/*
	if(!noclip_){
		camSpeed_ -= 0.004f;
		camera_.pos.setZ(camera_.pos[2] + camSpeed_);

		if(camera_.pos[2] <= 0){
			camera_.pos.setZ(0);
			camSpeed_ = 0;

			if(window_.isKeyDown(GLFW_KEY_SPACE))
				camSpeed_ = 0.07f;
		}
	}
	else{
		camSpeed_ = 0;
		if(window_.isKeyDown(GLFW_KEY_Q))	camera_.pos.setZ(camera_.pos[2] + camSpeed);
		if(window_.isKeyDown(GLFW_KEY_E))	camera_.pos.setZ(camera_.pos[2] - camSpeed);
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
