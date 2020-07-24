#pragma once

/*
 *	quaternion.h
 *
 *	Quaternion class and functions.
 *
 */

class Quaternion;

#include"vec3.h"


class Quaternion{

	float x_;
	float y_;
	float z_;
	float w_;

public:
	Quaternion(float x, float y, float z, float w);
	Quaternion();


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
