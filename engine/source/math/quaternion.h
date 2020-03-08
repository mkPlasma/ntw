#pragma once

/*
 *	quaternion.h
 *
 *	Quaternion class and functions.
 *
 */

#include"vec3.h"


class Quaternion{

	float w_;
	float x_;
	float y_;
	float z_;

public:
	Quaternion();
	Quaternion(float w, float x, float y, float z);


	friend Quaternion operator+(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator-(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator-(const Quaternion& a);
	friend Quaternion operator*(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator*(const Quaternion& a, float b);
	friend Quaternion operator/(const Quaternion& a, float b);
	friend bool operator==(const Quaternion& a, const Quaternion& b);

	Quaternion& operator+=(const Quaternion& a);
	Quaternion& operator-=(const Quaternion& a);
	Quaternion& operator*=(const Quaternion& a);
	Quaternion& operator*=(float a);
	Quaternion& operator/=(float a);

	float operator[](int a) const;


	void setRotation(Vec3 axis, float ang);
	void setRotation(float x, float y, float z, float ang);
	void setRotation(const Vec3& euler);
	void setRotation(float x, float y, float z);
	Quaternion& rotate(const Vec3& axis, float ang);
	Quaternion& rotate(float x, float y, float z, float ang);
	Quaternion& rotate(const Vec3& euler);
	Quaternion& rotate(float x, float y, float z);


	float magnitude() const;
	Quaternion unitQuaternion() const;
	Quaternion& normalize();
};
