#pragma once

/*
 *	portal.h
 *
 *	Planar portal that links two sections of the world.
 *
 */

class Portal;

#include"math/vec3.h"
#include"math/matrix.h"
#include"math/quaternion.h"
#include"objects/collider.h"
#include<vector>

using std::vector;

struct Collider;


class Portal{
	struct ClipPlane{
		Vec3 position;
		Vec3 normal;
	};

	int portalNum_;
	int portalNumBack_;

	Portal* pairedPortal_;
	Portal* pairedPortalBack_;
	
	Vec3 position_;
	Quaternion rotation_;
	Vec3 normal_;

	float width_;
	float height_;

	vector<Vec3> vertices_;

	Matrix transformationMatrix_;
	Matrix rotationMatrix_;

	vector<ClipPlane> clipPlanes_;

	Collider collider_;

public:
	Portal(Vec3 position, Quaternion rotation, float width, float height, int portalNum, int portalNumBack = -1);

	void update();

	bool isPointInFront(const Vec3& v);
	bool isPointWithinClipPlanes(const Vec3& v);

	Vec3 getTransformedVector(const Vec3& v);
	Vec3 getRotatedVector(const Vec3& v);


	void setPairedPortal(Portal* pairedPortal);
	void setPairedPortalBack(Portal* pairedPortalBack);

	void setPosition(const Vec3& position);
	void setRotation(const Quaternion& rotation);

	void setWidth(float width);
	void setHeight(float height);


	int getPortalNum() const;
	int getPortalNumBack() const;

	Portal* getPairedPortal() const;
	Portal* getPairedPortalBack() const;

	const Vec3& getPosition() const;
	const Quaternion& getRotation() const;
	const Vec3& getNormal() const;

	float getWidth() const;
	float getHeight() const;

	const Matrix& getTransformationMatrix() const;
	const Matrix& getRotationMatrix() const;

	const vector<Vec3>& getVertices() const;
	const Collider& getCollider() const;
};
