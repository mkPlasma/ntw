#include"quaternion.h"

#include<stdexcept>
#include<string>
#include"mathFunc.h"


Quaternion::Quaternion(float x, float y, float z, float w) :
	x_(x), y_(y), z_(z), w_(w) {

}

Quaternion::Quaternion() : Quaternion(0, 0, 0, 1) {

}


Quaternion operator+(const Quaternion& a, const Quaternion& b){
	return Quaternion(
		a[0] + b[0],
		a[1] + b[1],
		a[2] + b[2],
		a[3] + b[3]
	);
}

Quaternion operator-(const Quaternion& a, const Quaternion& b){
	return Quaternion(
		a[0] - b[0],
		a[1] - b[1],
		a[2] - b[2],
		a[3] - b[3]
	);
}

Quaternion operator-(const Quaternion& a){
	return Quaternion(
		-a[0],
		-a[1],
		-a[2],
		-a[3]
	);
}

Quaternion operator*(const Quaternion& a, const Quaternion& b){
	return Quaternion(
		a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
		a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
		a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
		a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]
	);
}

Quaternion operator*(const Quaternion& a, float b){
	return Quaternion(
		a[0] * b,
		a[1] * b,
		a[2] * b,
		a[3] * b
	);
}

Quaternion operator/(const Quaternion& a, float b){
	return Quaternion(
		a[0] / b,
		a[1] / b,
		a[2] / b,
		a[3] / b
	);
}

Quaternion& Quaternion::operator+=(const Quaternion& a){
	return *this = *this + a;
}

Quaternion& Quaternion::operator-=(const Quaternion& a){
	return *this = *this - a;
}

Quaternion& Quaternion::operator*=(const Quaternion& a){
	return *this = *this * a;
}

Quaternion& Quaternion::operator*=(float a){
	return *this = *this * a;
}

Quaternion& Quaternion::operator/=(float a){
	return *this = *this / a;
}

bool operator==(const Quaternion& a, const Quaternion& b){
	return	a[0] == b[0] &&
			a[1] == b[1] &&
			a[2] == b[2] &&
			a[3] == b[3];
}

float Quaternion::operator[](int a) const{
	switch(a){
	case 0:	return x_;
	case 1:	return y_;
	case 2:	return z_;
	case 3:	return w_;
	}

	throw std::runtime_error("Quaternion index out of bounds: " + std::to_string(a));
}


void Quaternion::setRotation(Vec3 axis, float ang){

	if(axis.magnitude() == 0)
		return;

	axis.normalize();
	axis *= sinf(ang / 2);

	x_ = axis[0];
	y_ = axis[1];
	z_ = axis[2];
	w_ = cosf(ang / 2);
}

void Quaternion::setRotation(float x, float y, float z, float ang){
	setRotation(Vec3(x, y, z), ang);
}

void Quaternion::setRotation(const Vec3& euler){
	
	float cx = cosf(ntw::toRadians(euler[0] / 2));
	float sx = sinf(ntw::toRadians(euler[0] / 2));
	float cy = cosf(ntw::toRadians(euler[1] / 2));
	float sy = sinf(ntw::toRadians(euler[1] / 2));
	float cz = cosf(ntw::toRadians(euler[2] / 2));
	float sz = sinf(ntw::toRadians(euler[2] / 2));

	x_ = sx * cy * cz - cx * sy * sz;
	y_ = sx * cy * sz + cx * sy * cz;
	z_ = cx * cy * sz - sx * sy * cz;
	w_ = cx * cy * cz + sx * sy * sz;
}

void Quaternion::setRotation(float x, float y, float z){
	setRotation(Vec3(x, y, z));
}

Quaternion& Quaternion::rotate(const Vec3& axis, float ang){

	Quaternion r;
	r.setRotation(axis, ang);

	return (*this = r * *this).normalize();
}

Quaternion& Quaternion::rotate(float x, float y, float z, float ang){
	return rotate(Vec3(x, y, z), ang);
}

Quaternion& Quaternion::rotate(const Vec3& euler){

	Quaternion r;
	r.setRotation(euler);

	return (*this = r * *this).normalize();
}

Quaternion& Quaternion::rotate(float x, float y, float z){
	return rotate(Vec3(x, y, z));
}


float Quaternion::magnitude() const{
	return sqrtf(
		powf(x_, 2) +
		powf(y_, 2) +
		powf(z_, 2) +
		powf(w_, 2)
	);
}

Quaternion Quaternion::unitQuaternion() const{
	return *this / magnitude();
}

Quaternion& Quaternion::normalize(){
	return *this /= magnitude();
}
