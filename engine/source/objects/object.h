#pragma once

/*
 *	object.h
 *
 *	Game object with a model and physics and transformation properties.
 *
 */

#include"math/vec3.h"
#include"math/quaternion.h"
#include"graphics/model.h"
#include"objects/material.h"
#include"graphics/renderType.h"
#include"physics/physEnum.h"
#include<al.h>


class Object{
protected:
	Vec3 position_;
	Vec3 scale_;
	Quaternion rotation_;

	Model* model_;
	Material* material_;

	RenderType renderType_;
	PhysicsType physicsType_;
	HitboxType hitboxType_;

	ALuint soundSource_;

public:
	Object(Model* model, Material* material, RenderType renderType, PhysicsType physicsType, HitboxType hitboxType);
	Object(Model* model, Material* material, RenderType renderType, HitboxType hitboxType = HitboxType::NONE);

	virtual void update();

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


	Vec3 getPosition() const;
	Vec3 getScale() const;
	Quaternion getRotation() const;

	Model* getModel() const;
	Material* getMaterial() const;

	RenderType getRenderType() const;
	PhysicsType getPhysicsType() const;
	HitboxType getHitboxType() const;

	ALuint getSoundSource() const;
	bool hasSoundSource() const;
};
