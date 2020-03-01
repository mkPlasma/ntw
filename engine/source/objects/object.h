#pragma once

/*
 *	object.h
 *
 *	Game object with a model and physics and transformation properties.
 *
 */

#include"graphics/model.h"
#include"physics/physEnum.h"
#include"math/vec3.h"
#include"math/quaternion.h"


enum RenderType{
	RENDER_NONE,
	RENDER_STATIC,
	RENDER_DYNAMIC
};

class Object{
protected:
	Model* model_;

	const int renderType_;
	const int physicsType_;
	const int hitboxType_;

	Vec3 position_;
	Vec3 scale_;
	Quaternion rotation_;

public:
	Object(Model* model, const int& renderType, const int& physicsType, const int& hitboxType);
	Object(Model* model, const int& renderType, const int& hitboxType = 0);


	void setPosition(const Vec3& position);
	void setPosition(const float& x, const float& y, const float& z);
	void move(const Vec3& pos);
	void move(const float& x, const float& y, const float& z);

	void setScale(const Vec3& scale);
	void setScale(const float& x, const float& y, const float& z);
	void setScale(const float& s);
	void scale(const Vec3& scale);
	void scale(const float& x, const float& y, const float& z);
	void scale(const float& s);

	void setRotation(const Quaternion& rotation);
	void setRotation(const Vec3& axis, const float& ang);
	void setRotation(const float& x, const float& y, const float& z, const float& ang);
	void setRotation(const Vec3& euler);
	void setRotation(const float& x, const float& y, const float& z);
	void rotate(const Quaternion& rotation);
	void rotate(const Vec3& axis, const float& ang);
	void rotate(const float& x, const float& y, const float& z, const float& ang);
	void rotate(const Vec3& euler);
	void rotate(const float& x, const float& y, const float& z);


	Model* getModel() const;
	const int& getRenderType() const;
	const int& getPhysicsType() const;
	const int& getHitboxType() const;

	Vec3 getPosition() const;
	Vec3 getScale() const;
	Quaternion getRotation() const;
};
