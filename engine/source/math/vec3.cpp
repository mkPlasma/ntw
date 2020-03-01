#include"vec3.h"

#include<stdexcept>
#include<string>
#include"mathFunc.h"


Vec3::Vec3() : x_(0), y_(0), z_(0) {}

Vec3::Vec3(const float& x, const float& y, const float& z) : x_(x), y_(y), z_(z) {}

// Direction vector
Vec3::Vec3(float yaw, float pitch){

	yaw		= ntw::toRadians(yaw);
	pitch	= ntw::toRadians(pitch - 90);

	float sp = sinf(pitch);
	x_ = cosf(yaw) * sp;
	y_ = sinf(yaw) * sp;
	z_ = cosf(pitch);
}


Vec3 operator+(const Vec3& a, const Vec3& b){
	return Vec3(
		a[0] + b[0],
		a[1] + b[1],
		a[2] + b[2]
	);
}

Vec3 operator+(const Vec3& a, const float& b){
	return Vec3(
		a[0] + b,
		a[1] + b,
		a[2] + b
	);
}

Vec3 operator-(const Vec3& a, const Vec3& b){
	return Vec3(
		a[0] - b[0],
		a[1] - b[1],
		a[2] - b[2]
	);
}

Vec3 operator-(const Vec3& a, const float& b){
	return Vec3(
		a[0] - b,
		a[1] - b,
		a[2] - b
	);
}

Vec3 operator-(const float& a, const Vec3& b){
	return Vec3(
		a - b[0],
		a - b[1],
		a - b[2]
	);
}

Vec3 operator-(const Vec3& a){
	return Vec3(
		-a[0],
		-a[1],
		-a[2]
	);
}

float operator*(const Vec3& a, const Vec3& b){
	return	(a[0] * b[0]) +
			(a[1] * b[1]) +
			(a[2] * b[2]);
}

Vec3 operator*(const Vec3& a, const float& b){
	return Vec3(
		a[0] * b,
		a[1] * b,
		a[2] * b
	);
}

Vec3 operator*(const float& a, const Vec3& b){
	return b * a;
}

Vec3 operator/(const Vec3& a, const float& b){
	return Vec3(
		a[0] / b,
		a[1] / b,
		a[2] / b
	);
}


bool operator==(const Vec3& a, const Vec3& b){
	return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

bool operator!=(const Vec3& a, const Vec3& b){
	return a[0] != b[0] || a[1] != b[1] || a[2] != b[2];
}


Vec3& Vec3::operator+=(const Vec3& a){
	return *this = *this + a;
}

Vec3& Vec3::operator+=(const float& a){
	return *this = *this + a;
}

Vec3& Vec3::operator-=(const Vec3& a){
	return *this = *this - a;
}

Vec3& Vec3::operator-=(const float& a){
	return *this = *this - a;
}

Vec3& Vec3::operator*=(const Vec3& a){
	x_ *= a[0];
	y_ *= a[1];
	z_ *= a[2];
	return *this;
}

Vec3& Vec3::operator*=(const float& a){
	return *this = *this * a;
}

Vec3& Vec3::operator/=(const float& a){
	return *this = *this / a;
}

Vec3 ntw::crossProduct(const Vec3& a, const Vec3& b){
	return Vec3(
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	);
}

const float& Vec3::operator[](const int& a) const{
	switch(a){
	case 0:	return x_;
	case 1:	return y_;
	case 2:	return z_;
	}

	throw std::runtime_error("Vector index out of bounds: " + std::to_string(a));
}


Vec3 Vec3::multiplyElementWise(const Vec3& a){
	Vec3 b = *this;
	return b *= a;
}

Vec3 Vec3::compOn(const Vec3& a) const{
	return *this * (*this * a / a.magnitude());
}

Vec3 Vec3::projOn(const Vec3& a) const{
	return *this * (*this * a / a.magnitude2());
}

bool Vec3::isZero() const{
	return x_ == 0 && y_ == 0 && z_ == 0;
}

bool Vec3::nonzero() const{
	return x_ != 0 || y_ != 0 || z_ != 0;
}

float Vec3::magnitude() const{
	return sqrtf(magnitude2());
}

float Vec3::magnitude2() const{
	return powf(x_, 2) + powf(y_, 2) + powf(z_, 2);
}

Vec3 Vec3::unitVector() const{
	return *this / magnitude();
}

Vec3& Vec3::normalize(){
	return *this /= magnitude();
}

bool Vec3::equalsWithinThreshold(const Vec3& a, const float& threshold){
	return
		abs(x_ - a[0]) <= threshold &&
		abs(y_ - a[1]) <= threshold &&
		abs(z_ - a[2]) <= threshold;
}

void Vec3::setX(const float& x){
	x_ = x;
}

void Vec3::setY(const float& y){
	y_ = y;
}

void Vec3::setZ(const float& z){
	z_ = z;
}

float Vec3::x() const{
	return x_;
}

float Vec3::y() const{
	return y_;
}

float Vec3::z() const{
	return z_;
}
