#include"object.h"

#include"math/matrix.h"


Object::Object(Model* model, Material* material, RenderType renderType, PhysicsType physicsType, HitboxType hitboxType) :
	scale_(Vec3(1, 1, 1)), model_(model), material_(material), renderType_(renderType),
	physicsType_(physicsType), hitboxType_(hitboxType), hitboxCached_(false), soundSource_(-1) {

}

Object::Object(Model* model, Material* material, RenderType renderType, HitboxType hitboxType) :
	Object(model, material, renderType, hitboxType == HitboxType::NONE ? PhysicsType::NONE : PhysicsType::STATIC, hitboxType) {
	
}

void Object::update(float timeDelta){

	// Update sound source position
	updateSoundSource();

	// Reset hitbox cache for next frame
	hitboxCached_ = false;
}


void Object::updateSoundSource(){
	if(hasSoundSource())
		alSource3f(soundSource_, AL_POSITION, position_[0], position_[1], position_[2]);
}

void Object::playSound(ALuint soundID){

	if(!hasSoundSource() || soundID == -1)
		return;

	alSourcei(soundSource_, AL_BUFFER, soundID);
	alSourcePlay(soundSource_);
}

void Object::createSoundSource(){
	if(!hasSoundSource())
		alGenBuffers(1, &soundSource_);
}

void Object::deleteSoundSource(){
	if(hasSoundSource())
		alDeleteSources(1, &soundSource_);
}


void Object::setPosition(const Vec3& position){
	position_ = position;
}

void Object::setPosition(float x, float y, float z){
	position_.setX(x);
	position_.setY(y);
	position_.setZ(z);
}

void Object::move(const Vec3& pos){
	position_ += pos;
}

void Object::move(float x, float y, float z){
	position_ += Vec3(x, y, z);
}

void Object::setScale(const Vec3& scale){
	scale_ = scale;
}

void Object::setScale(float x, float y, float z){
	scale_.setX(x);
	scale_.setY(y);
	scale_.setZ(z);
}

void Object::setScale(float s){
	scale_.setX(s);
	scale_.setY(s);
	scale_.setZ(s);
}

void Object::scale(const Vec3& scale){
	scale_.setX(scale_[0] * scale[0]);
	scale_.setY(scale_[1] * scale[1]);
	scale_.setZ(scale_[2] * scale[2]);
}

void Object::scale(float x, float y, float z){
	scale_.setX(scale_[0] * x);
	scale_.setY(scale_[1] * y);
	scale_.setZ(scale_[2] * z);
}

void Object::scale(float s){
	scale_.setX(scale_[0] * s);
	scale_.setY(scale_[1] * s);
	scale_.setZ(scale_[2] * s);
}

void Object::setRotation(const Quaternion& rotation){
	rotation_ = rotation;
}

void Object::setRotation(const Vec3& axis, float ang){
	rotation_.setRotation(axis, ang);
}

void Object::setRotation(float x, float y, float z, float ang){
	rotation_.setRotation(x, y, z, ang);
}

void Object::setRotation(const Vec3& euler){
	rotation_.setRotation(euler);
}

void Object::setRotation(float x, float y, float z){
	rotation_.setRotation(x, y, z);
}

void Object::rotate(const Quaternion& rotation){
	rotation_ *= rotation;
	rotation_.normalize();
}

void Object::rotate(const Vec3& axis, float ang){
	rotation_.rotate(axis, ang);
}

void Object::rotate(float x, float y, float z, float ang){
	rotation_.rotate(x, y, z, ang);
}

void Object::rotate(const Vec3& euler){
	rotation_.rotate(euler);
}

void Object::rotate(float x, float y, float z){
	rotation_.rotate(x, y, z);
}


void Object::setModel(Model* model){
	model_ = model;
}

void Object::setMaterial(Material* material){
	material_ = material;
}


const Vec3& Object::getPosition() const{
	return position_;
}

const Vec3& Object::getScale() const{
	return scale_;
}

const Quaternion& Object::getRotation() const{
	return rotation_;
}

const Vec3& Object::getTPosition() const{
	return position_;
}

const Quaternion& Object::getTRotation() const{
	return rotation_;
}

Model* Object::getModel() const{
	return model_;
}

Material* Object::getMaterial() const{
	return material_;
}

RenderType Object::getRenderType() const{
	return renderType_;
}

PhysicsType Object::getPhysicsType() const{
	return physicsType_;
}

HitboxType Object::getHitboxType() const{
	return hitboxType_;
}

const vector<Vec3>& Object::getTransformedHitbox(){

	// If hitbox has not been calculated for this frame, do so
	if(!hitboxCached_ && model_ != nullptr){

		transformedHitbox_.clear();

		Matrix rotation = Matrix(3, 3, true).rotate(getTRotation());

		// For each vertex
		for(int i = 0; i < model_->hitbox.size(); i++){
			Vec3 v = model_->hitbox[i];

			// Scale
			v = v.multiplyElementWise(getScale());

			// Rotate
			v = rotation * v;

			// Translate
			v += getTPosition();

			transformedHitbox_.push_back(v);
		}

		hitboxCached_ = true;
	}

	return transformedHitbox_;
}

const Hitbox& Object::getTransformedHitboxSAT(){

	// TODO: don't cache hitbox again if position/rotation/scale have not changed since last cache

	// Skip if hitbox has already been cached
	if(hitboxCached_ || model_ == nullptr)
		return transformedHitboxSAT_;

	// Clear vertex and face data
	transformedHitboxSAT_.vertices.clear();
	transformedHitboxSAT_.faces.clear();

	// Copy edge data
	transformedHitboxSAT_.edges = model_->hitboxSAT.edges;

	Matrix rotation = Matrix(3, 3, true).rotate(getTRotation());

	// For each vertex
	for(Vec3 v : model_->hitboxSAT.vertices){

		// Scale
		v = v.multiplyElementWise(getScale());

		// Rotate
		v = rotation * v;

		// Translate
		v += getTPosition();

		transformedHitboxSAT_.vertices.push_back(v);
	}

	// For each face
	for(SATFace f : model_->hitboxSAT.faces){

		// Scale
		f.position = f.position.multiplyElementWise(getScale());

		// Rotate
		f.position = rotation * f.position;
		f.normal = rotation * f.normal;

		// Translate
		f.position += getTPosition();

		transformedHitboxSAT_.faces.push_back(f);
	}

	hitboxCached_ = true;

	return transformedHitboxSAT_;
}

ALuint Object::getSoundSource() const{
	return soundSource_;
}

bool Object::hasSoundSource() const{
	return soundSource_ != -1;
}
