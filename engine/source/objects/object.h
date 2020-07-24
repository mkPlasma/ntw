#pragma once

/*
 *	object.h
 *
 *	Game object with a model and physics and transformation properties.
 *
 */

class Object;

#include"math/vec3.h"
#include"math/quaternion.h"
#include"objects/model.h"
#include"objects/material.h"
#include"graphics/renderType.h"
#include"physics/physEnum.h"
#include"physics/physStruct.h"
#include"objects/collider.h"
#include<al.h>

class World;
struct Collider;
struct ContactInfo;


class Object{
protected:
	World& world_;

	Vec3 position_;
	Vec3 scale_;
	Quaternion rotation_;

	Model* model_;
	Material* material_;

	RenderType renderType_;
	PhysicsType physicsType_;
	HitboxType hitboxType_;

	bool deleted_;

	vector<Collider> colliders_;
	Hitbox transformedHitboxSAT_;
	bool hitboxCached_;

	vector<ContactInfo> contacts_;

	ALuint soundSource_;

public:
	Object(World& world, Model* model, Material* material, RenderType renderType, PhysicsType physicsType, HitboxType hitboxType);
	Object(World& world, Model* model, Material* material, RenderType renderType, HitboxType hitboxType = HitboxType::NONE);

	virtual void update(float timeDelta);

	void deleteObject();

	void updateSoundSource();
	void playSound(ALuint soundID);
	void createSoundSource();
	void deleteSoundSource();


	void setPosition(const Vec3& position);
	void setPosition(float x, float y, float z);
	void move(const Vec3& pos);
	void move(float x, float y, float z);

	void setScale(const Vec3& scale);
	void setScale(float x, float y, float z);
	void setScale(float s);
	void scale(const Vec3& scale);
	void scale(float x, float y, float z);
	void scale(float s);

	void setRotation(const Quaternion& rotation);
	void setRotation(const Vec3& axis, float ang);
	void setRotation(float x, float y, float z, float ang);
	void setRotation(const Vec3& euler);
	void setRotation(float x, float y, float z);
	void rotate(const Quaternion& rotation);
	void rotate(const Vec3& axis, float ang);
	void rotate(float x, float y, float z, float ang);
	void rotate(const Vec3& euler);
	void rotate(float x, float y, float z);


	void setModel(Model* model);
	void setMaterial(Material* material);

	void setRenderType(RenderType renderType);
	void setPhysicsType(PhysicsType physicsType);
	void setHitboxType(HitboxType hitboxType);

	bool cacheTransformedHitbox();

	void addContact(ContactInfo contact);



	const Vec3& getPosition() const;
	const Vec3& getScale() const;
	const Quaternion& getRotation() const;
	virtual const Vec3& getTPosition() const;
	virtual const Quaternion& getTRotation() const;

	Model* getModel() const;
	Material* getMaterial() const;

	RenderType getRenderType() const;
	PhysicsType getPhysicsType() const;
	HitboxType getHitboxType() const;

	const vector<Collider>& getColliders() const;
	const Hitbox& getTransformedHitboxSAT();


	bool isDeleted() const;

	ALuint getSoundSource() const;
	bool hasSoundSource() const;

	const vector<ContactInfo>& getContacts() const;
};
