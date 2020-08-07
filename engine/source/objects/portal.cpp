#include"portal.h"


Portal::Portal(Vec3 position, Quaternion rotation, float width, float height, int portalNum, int portalNumBack) :
	position_(position), rotation_(rotation), width_(width), height_(height),
	portalNum_(portalNum), portalNumBack_(portalNumBack), pairedPortal_(nullptr) {

	// Collider is purely for assigning AABB to portal and does not require a hitbox
	collider_.portal = this;
}

void Portal::update(){

	// Rotation matrix
	Matrix rotationMatrix = Matrix(3, 3, true).rotate(rotation_);

	// Normal vector
	normal_	= rotationMatrix * Vec3(0, 1, 0);


	// Matrices to transform vectors to paired portal
	transformationMatrix_ = Matrix();

	if(pairedPortal_){
		// World to portal space
		transformationMatrix_.translate(-position_);
		transformationMatrix_.rotate(-rotation_);

		// Mirroring
		Matrix mirroringMatrix = Matrix().rotate(Vec3(0, 0, 1), 180.0f, true);
		transformationMatrix_	= mirroringMatrix * transformationMatrix_;

		// Paired portal space to world
		transformationMatrix_.rotate(pairedPortal_->getRotation());
		transformationMatrix_.translate(pairedPortal_->getPosition());

		// Rotation only matrix
		rotationMatrix_ = transformationMatrix_;
		rotationMatrix_.set(0, 3, 0);
		rotationMatrix_.set(1, 3, 0);
		rotationMatrix_.set(2, 3, 0);
	}


	// Base vertices
	vertices_.clear();
	float hw = width_ / 2;
	float hh = height_ / 2;
	vertices_.push_back(Vec3(hw, 0, hh));
	vertices_.push_back(Vec3(hw, 0, -hh));
	vertices_.push_back(Vec3(-hw, 0, -hh));
	vertices_.push_back(Vec3(-hw, 0, hh));

	// Rotate and translate vertices
	for(Vec3& v : vertices_){
		v = rotationMatrix * v;
		v += position_;
	}


	// Get clip planes
	clipPlanes_.clear();

	for(int i = 0; i < 4; i++){
		
		// Second edge vertex
		int j = i == 3 ? 0 : i + 1;

		// Get edge vertices
		const Vec3& v1 = vertices_[i];
		const Vec3& v2 = vertices_[j];

		ClipPlane plane;
		plane.position = (v1 + v2) / 2;
		plane.normal = ntw::crossProduct(v1 - v2, normal_);

		// Check normal direction
		if((position_ - plane.position) * plane.normal < 0)
			plane.normal = -plane.normal;

		clipPlanes_.push_back(plane);
	}
}

bool Portal::isPointInFront(const Vec3& v){
	return (v - position_) * normal_ > 0;
}

bool Portal::isPointWithinClipPlanes(const Vec3& v){

	// Check which side of clip plane point is on
	for(const ClipPlane& plane : clipPlanes_)
		if((v - plane.position) * plane.normal < 0)
			return false;

	return true;
}

Vec3 Portal::getTransformedVector(const Vec3& v){
	return transformationMatrix_ * v;
}

Vec3 Portal::getRotatedVector(const Vec3& v){
	return rotationMatrix_ * v;
}


void Portal::setPairedPortal(Portal* pairedPortal){
	pairedPortal_ = pairedPortal;
}

void Portal::setPairedPortalBack(Portal* pairedPortalBack){
	pairedPortalBack_ = pairedPortalBack;
}

void Portal::setPosition(const Vec3& position){
	position_ = position;
}

void Portal::setRotation(const Quaternion& rotation){
	rotation_ = rotation;
}

void Portal::setWidth(float width){
	width_ = width;
}

void Portal::setHeight(float height){
	height_ = height;
}


int Portal::getPortalNum() const{
	return portalNum_;
}

int Portal::getPortalNumBack() const{
	return portalNumBack_;
}

Portal* Portal::getPairedPortal() const{
	return pairedPortal_;
}

Portal* Portal::getPairedPortalBack() const{
	return pairedPortalBack_;
}

const Vec3& Portal::getPosition() const{
	return position_;
}

const Quaternion& Portal::getRotation() const{
	return rotation_;
}

const Vec3& Portal::getNormal() const{
	return normal_;
}

float Portal::getWidth() const{
	return width_;
}

float Portal::getHeight() const{
	return height_;
}

const Matrix& Portal::getTransformationMatrix() const{
	return transformationMatrix_;
}

const Matrix& Portal::getRotationMatrix() const{
	return rotationMatrix_;
}

const vector<Vec3>& Portal::getVertices() const{
	return vertices_;
}

const Collider& Portal::getCollider() const{
	return collider_;
}
