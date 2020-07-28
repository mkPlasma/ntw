#pragma once

/*
 *	vec3.h
 *
 *	3-dimensional vector class.
 *
 */


class Vec3{

	float x_;
	float y_;
	float z_;

public:
	Vec3();
	Vec3(float val);
	Vec3(float x, float y, float z);
	Vec3(float yaw, float pitch);

	friend Vec3 operator+(const Vec3& a, const Vec3& b);
	friend Vec3 operator+(const Vec3& a, float b);
	friend Vec3 operator-(const Vec3& a, const Vec3& b);
	friend Vec3 operator-(const Vec3& a, float b);
	friend Vec3 operator-(const Vec3& a);
	friend float operator*(const Vec3& a, const Vec3& b);
	friend Vec3 operator*(const Vec3& a, float b);
	friend Vec3 operator*(float a, const Vec3& b);
	friend Vec3 operator/(const Vec3& a, float b);

	friend bool operator==(const Vec3& a, const Vec3& b);
	friend bool operator!=(const Vec3& a, const Vec3& b);

	Vec3& operator=(float a);
	Vec3& operator+=(const Vec3& a);
	Vec3& operator+=(float a);
	Vec3& operator-=(const Vec3& a);
	Vec3& operator-=(float a);
	Vec3& operator*=(const Vec3& a);
	Vec3& operator*=(float a);
	Vec3& operator/=(float a);

	float operator[](int a) const;
	float& operator[](int a);

	Vec3 multiplyElementWise(const Vec3& a);

	float compOn(const Vec3& a) const;
	Vec3 projOn(const Vec3& a) const;
	Vec3 clampedProjOn(const Vec3& a) const;

	bool isZero() const;
	bool nonzero() const;
	bool isNan() const;
	float magnitude() const;
	float magnitude2() const;

	Vec3 unitVector() const;
	Vec3& normalize();
	Vec3& setMagnitude(float magnitude);

	bool equalsWithinThreshold(const Vec3& a, float threshold) const;


	void setX(float x);
	void setY(float y);
	void setZ(float z);

	float x() const;
	float y() const;
	float z() const;
};

namespace ntw{
	Vec3 crossProduct(const Vec3& a, const Vec3& b);
}
