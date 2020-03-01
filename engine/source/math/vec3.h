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
	Vec3(const float& x, const float& y, const float& z);
	Vec3(float yaw, float pitch);

	friend Vec3 operator+(const Vec3& a, const Vec3& b);
	friend Vec3 operator+(const Vec3& a, const float& b);
	friend Vec3 operator-(const Vec3& a, const Vec3& b);
	friend Vec3 operator-(const Vec3& a, const float& b);
	friend Vec3 operator-(const Vec3& a);
	friend float operator*(const Vec3& a, const Vec3& b);
	friend Vec3 operator*(const Vec3& a, const float& b);
	friend Vec3 operator*(const float& a, const Vec3& b);
	friend Vec3 operator/(const Vec3& a, const float& b);

	friend bool operator==(const Vec3& a, const Vec3& b);
	friend bool operator!=(const Vec3& a, const Vec3& b);

	Vec3& operator+=(const Vec3& a);
	Vec3& operator+=(const float& a);
	Vec3& operator-=(const Vec3& a);
	Vec3& operator-=(const float& a);
	Vec3& operator*=(const Vec3& a);
	Vec3& operator*=(const float& a);
	Vec3& operator/=(const float& a);

	const float& operator[](const int& a) const;

	Vec3 multiplyElementWise(const Vec3& a);

	Vec3 compOn(const Vec3& a) const;
	Vec3 projOn(const Vec3& a) const;

	bool isZero() const;
	bool nonzero() const;
	float magnitude() const;
	float magnitude2() const;

	Vec3 unitVector() const;
	Vec3& normalize();

	bool equalsWithinThreshold(const Vec3& a, const float& threshold);


	void setX(const float& x);
	void setY(const float& y);
	void setZ(const float& z);

	float x() const;
	float y() const;
	float z() const;
};

namespace ntw{
	Vec3 crossProduct(const Vec3& a, const Vec3& b);
}
