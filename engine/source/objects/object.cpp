#include"object.h"


Object::Object(Model* model, const int& renderType, const int& physicsType, const int& hitboxType) :
	model_(model), renderType_(renderType), physicsType_(physicsType), hitboxType_(hitboxType), scale_(Vec3(1, 1, 1)) {

}

Object::Object(Model* model, const int& renderType, const int& hitboxType) :
	Object(model, renderType, hitboxType == HITBOX_NONE ? PHYS_NONE : PHYS_STATIC, hitboxType) {
	
}

void Object::setPosition(const Vec3& position){
	position_ = position;
}

void Object::setPosition(const float& x, const float& y, const float& z){
	position_.setX(x);
	position_.setY(y);
	position_.setZ(z);
}

void Object::move(const Vec3& pos){
	position_ += pos;
}

void Object::move(const float& x, const float& y, const float& z){
	position_ += Vec3(x, y, z);
}

void Object::setScale(const Vec3& scale){
	scale_ = scale;
}

void Object::setScale(const float& x, const float& y, const float& z){
	scale_.setX(x);
	scale_.setY(y);
	scale_.setZ(z);
}

void Object::setScale(const float& s){
	scale_.setX(s);
	scale_.setY(s);
	scale_.setZ(s);
}

void Object::scale(const Vec3& scale){
	scale_.setX(scale_[0] * scale[0]);
	scale_.setY(scale_[1] * scale[1]);
	scale_.setZ(scale_[2] * scale[2]);
}

void Object::scale(const float& x, const float& y, const float& z){
	scale_.setX(scale_[0] * x);
	scale_.setY(scale_[1] * y);
	scale_.setZ(scale_[2] * z);
}

void Object::scale(const float& s){
	scale_.setX(scale_[0] * s);
	scale_.setY(scale_[1] * s);
	scale_.setZ(scale_[2] * s);
}

void Object::setRotation(const Quaternion& rotation){
	rotation_ = rotation;
}

void Object::setRotation(const Vec3& axis, const float& ang){
	rotation_.setRotation(axis, ang);
}

void Object::setRotation(const float& x, const float& y, const float& z, const float& ang){
	rotation_.setRotation(x, y, z, ang);
}

void Object::setRotation(const Vec3& euler){
	rotation_.setRotation(euler);
}

void Object::setRotation(const float& x, const float& y, const float& z){
	rotation_.setRotation(x, y, z);
}

void Object::rotate(const Quaternion& rotation){
	rotation_ *= rotation;
	rotation_.normalize();
}

void Object::rotate(const Vec3& axis, const float& ang){
	rotation_.rotate(axis, ang);
}

void Object::rotate(const float& x, const float& y, const float& z, const float& ang){
	rotation_.rotate(x, y, z, ang);
}

void Object::rotate(const Vec3& euler){
	rotation_.rotate(euler);
}

void Object::rotate(const float& x, const float& y, const float& z){
	rotation_.rotate(x, y, z);
}


Model* Object::getModel() const{
	return model_;
}

const int& Object::getRenderType() const{
	return renderType_;
}

const int& Object::getPhysicsType() const{
	return physicsType_;
}

const int& Object::getHitboxType() const{
	return hitboxType_;
}


Vec3 Object::getPosition() const{
	return position_;
}

Vec3 Object::getScale() const{
	return scale_;
}

Quaternion Object::getRotation() const{
	return rotation_;
}
