#include"object.h"

#include"math/matrix.h"


Object::Object(World& world, Model* model, Material* material, RenderType renderType, PhysicsType physicsType) :
	world_(world), scale_(Vec3(1, 1, 1)), model_(model), material_(material), renderType_(renderType),
	physicsType_(physicsType), hitboxCached_(false), soundSource_(-1) {

	// Add colliders
	if(physicsType != PhysicsType::NONE && model_ != nullptr){
		for(Hitbox& hitbox : model_->colliderHitboxes){
			Collider c;
			c.hitbox = &hitbox;
			c.parent = this;

			// Copy hitbox data to transformed hitbox
			c.hitboxTransformed.vertices	= c.hitbox->vertices;
			c.hitboxTransformed.edges		= c.hitbox->edges;
			c.hitboxTransformed.faces		= c.hitbox->faces;

			colliders_.push_back(c);
		}
	}
}

void Object::update(float timeDelta){

	// Update sound source position
	updateSoundSource();

	// Clear contacts
	contacts_.clear();
}


void Object::deleteObject(){
	deleted_ = true;
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
	rotation_ = rotation * rotation_;
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

void Object::setRenderType(RenderType renderType){
	renderType_ = renderType;
}

void Object::setPhysicsType(PhysicsType physicsType){
	physicsType_ = physicsType;
}

bool Object::cacheTransformedHitbox(){

	// Skip if hitbox has already been cached
	if(hitboxCached_ || colliders_.empty())
		return false;

	Matrix rotation = Matrix(3, 3, true).rotate(getTRotation());

	for(Collider& collider : colliders_){

		// For each vertex
		for(int i = 0; i < collider.hitbox->vertices.size(); i++){

			Vec3& v = collider.hitboxTransformed.vertices[i];
			v = collider.hitbox->vertices[i];

			// Scale
			v *= getScale();

			// Rotate
			v = rotation * v;

			// Translate
			v += getTPosition();
		}

		// For each face
		for(int i = 0; i < collider.hitbox->faces.size(); i++){

			SATFace& f = collider.hitboxTransformed.faces[i];
			f = collider.hitbox->faces[i];

			// Scale
			f.position *= getScale();

			// Rotate
			f.position = rotation * f.position;
			f.normal = rotation * f.normal;

			// Translate
			f.position += getTPosition();
		}
	}

	hitboxCached_ = true;
	return true;
}

void Object::addContact(ObjectContactInfo contact){
	contacts_.push_back(contact);
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

bool Object::isPlayer() const{
	return false;
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

const vector<Collider>& Object::getColliders() const{
	return colliders_;
}


bool Object::isDeleted() const{
	return deleted_;
}

ALuint Object::getSoundSource() const{
	return soundSource_;
}

bool Object::hasSoundSource() const{
	return soundSource_ != -1;
}

const vector<ObjectContactInfo>& Object::getContacts() const{
	return contacts_;
}
