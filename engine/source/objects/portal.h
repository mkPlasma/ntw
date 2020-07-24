#pragma once

/*
 *	portal.h
 *
 *	Planar portal that links two sections of the world.
 *
 */

class Portal;

#include"math/vec3.h"
#include<vector>

using std::vector;


class Portal{

	int portalNum_;
	int portalNumBack_;

	Portal* pairedPortal_;
	Portal* pairedPortalBack_;
	
	Vec3 position_;
	Vec3 rotation_;
	Vec3 normal_;

	float width_;
	float height_;

	vector<Vec3> verts_;

public:
	Portal(Vec3 position, Vec3 rotation, float width, float height, int portalNum, int portalNumBack = -1);

	void updateGeometry();


	void setPairedPortal(Portal* pairedPortal);
	void setPairedPortalBack(Portal* pairedPortalBack);

	void setPosition(const Vec3& position);
	void setRotation(const Vec3& rotation);

	void setWidth(float width);
	void setHeight(float height);


	int getPortalNum();
	int getPortalNumBack();

	Portal* getPairedPortal();
	Portal* getPairedPortalBack();

	const Vec3& getPosition();
	const Vec3& getRotation();
	const Vec3& getNormal();

	float getWidth();
	float getHeight();

	const vector<Vec3>& getVerts();
};
