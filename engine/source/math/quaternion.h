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
	Quaternion(const float& w, const float& x, const float& y, const float& z);


	friend Quaternion operator+(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator-(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator-(const Quaternion& a);
	friend Quaternion operator*(const Quaternion& a, const Quaternion& b);
	friend Quaternion operator*(const Quaternion& a, const float& b);
	friend Quaternion operator/(const Quaternion& a, const float& b);
	friend bool operator==(const Quaternion& a, const Quaternion& b);

	Quaternion& operator+=(const Quaternion& a);
	Quaternion& operator-=(const Quaternion& a);
	Quaternion& operator*=(const Quaternion& a);
	Quaternion& operator*=(const float& a);
	Quaternion& operator/=(const float& a);

	float operator[](const int& a) const;


	void setRotation(Vec3 axis, const float& ang);
	void setRotation(const float& x, const float& y, const float& z, const float& ang);
	void setRotation(const Vec3& euler);
	void setRotation(const float& x, const float& y, const float& z);
	Quaternion& rotate(const Vec3& axis, const float& ang);
	Quaternion& rotate(const float& x, const float& y, const float& z, const float& ang);
	Quaternion& rotate(const Vec3& euler);
	Quaternion& rotate(const float& x, const float& y, const float& z);


	float magnitude() const;
	Quaternion unitQuaternion() const;
	Quaternion& normalize();
};
