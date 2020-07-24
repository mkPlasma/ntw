#include"portal.h"

#include"math/matrix.h"


Portal::Portal(Vec3 position, Vec3 rotation, float width, float height, int portalNum, int portalNumBack) :
	position_(position), rotation_(rotation), width_(width), height_(height),
	portalNum_(portalNum), portalNumBack_(portalNumBack), pairedPortal_(nullptr) {

	updateGeometry();
}

void Portal::updateGeometry(){

	verts_.clear();

	// Base vertices
	float hw = width_ / 2;
	float hh = height_ / 2;
	verts_.push_back(Vec3(hw, hh, 0));
	verts_.push_back(Vec3(hw, -hh, 0));
	verts_.push_back(Vec3(-hw, hh, 0));
	verts_.push_back(Vec3(-hw, -hh, 0));

	// Rotate
	Matrix rotation = Matrix(3, 3, true);
	rotation.rotate(rotation_);

	// Normal
	normal_ = rotation * Vec3(0, 0, 1);

	// Rotate and translate vertices
	for(Vec3& v : verts_){
		v = rotation * v;
		v += position_;
	}
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

void Portal::setRotation(const Vec3& rotation){
	rotation_ = rotation;
}

void Portal::setWidth(float width){
	width_ = width;
}

void Portal::setHeight(float height){
	height_ = height;
}


int Portal::getPortalNum(){
	return portalNum_;
}

int Portal::getPortalNumBack(){
	return portalNumBack_;
}

Portal* Portal::getPairedPortal(){
	return pairedPortal_;
}

Portal* Portal::getPairedPortalBack(){
	return pairedPortalBack_;
}

const Vec3& Portal::getPosition(){
	return position_;
}

const Vec3& Portal::getRotation(){
	return rotation_;
}

const Vec3& Portal::getNormal(){
	return normal_;
}

float Portal::getWidth(){
	return width_;
}

float Portal::getHeight(){
	return height_;
}

const vector<Vec3>& Portal::getVerts(){
	return verts_;
}
