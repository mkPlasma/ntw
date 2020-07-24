#include"vec3.h"

#include<stdexcept>
#include<string>
#include"mathFunc.h"


Vec3::Vec3() : x_(0), y_(0), z_(0) {}

Vec3::Vec3(float x, float y, float z) : x_(x), y_(y), z_(z) {}

// Direction vector
Vec3::Vec3(float yaw, float pitch){

	yaw		= ntw::toRadians(yaw);
	pitch	= ntw::toRadians(90 - pitch);

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

Vec3 operator+(const Vec3& a, float b){
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

Vec3 operator-(const Vec3& a, float b){
	return Vec3(
		a[0] - b,
		a[1] - b,
		a[2] - b
	);
}

Vec3 operator-(float a, const Vec3& b){
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

Vec3 operator*(const Vec3& a, float b){
	return Vec3(
		a[0] * b,
		a[1] * b,
		a[2] * b
	);
}

Vec3 operator*(float a, const Vec3& b){
	return b * a;
}

Vec3 operator/(const Vec3& a, float b){
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


Vec3& Vec3::operator=(float a){
	x_ = y_ = z_ = a;
	return *this;
}

Vec3& Vec3::operator+=(const Vec3& a){
	return *this = *this + a;
}

Vec3& Vec3::operator+=(float a){
	return *this = *this + a;
}

Vec3& Vec3::operator-=(const Vec3& a){
	return *this = *this - a;
}

Vec3& Vec3::operator-=(float a){
	return *this = *this - a;
}

Vec3& Vec3::operator*=(const Vec3& a){
	x_ *= a[0];
	y_ *= a[1];
	z_ *= a[2];
	return *this;
}

Vec3& Vec3::operator*=(float a){
	return *this = *this * a;
}

Vec3& Vec3::operator/=(float a){
	return *this = *this / a;
}

Vec3 ntw::crossProduct(const Vec3& a, const Vec3& b){
	return Vec3(
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	);
}

float Vec3::operator[](int a) const{
	switch(a){
	case 0:	return x_;
	case 1:	return y_;
	case 2:	return z_;
	}

	throw std::runtime_error("Vector index out of bounds: " + std::to_string(a));
}

float& Vec3::operator[](int a){
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

float Vec3::compOn(const Vec3& a) const{
	return (*this * a / a.magnitude());
}

Vec3 Vec3::projOn(const Vec3& a) const{
	return a * (*this * a / a.magnitude2());
}

Vec3 Vec3::clampedProjOn(const Vec3& a) const{

	// Standard projection
	Vec3 proj = projOn(a);

	// Clamp components to zero if the two vectors are in opposite directions
	Vec3 comp = *this;
	comp *= a;

	if(comp[0] < 0)	proj[0] = 0;
	if(comp[1] < 0)	proj[1] = 0;
	if(comp[2] < 0)	proj[2] = 0;

	return proj;
}

bool Vec3::isZero() const{
	return x_ == 0 && y_ == 0 && z_ == 0;
}

bool Vec3::nonzero() const{
	return x_ != 0 || y_ != 0 || z_ != 0;
}

bool Vec3::isNan() const{
	return isnan(x_) || isnan(y_) || isnan(z_);
}

float Vec3::magnitude() const{
	return sqrtf(magnitude2());
}

float Vec3::magnitude2() const{
	return powf(x_, 2) + powf(y_, 2) + powf(z_, 2);
}

Vec3 Vec3::unitVector() const{
	float mag = magnitude();
	return mag == 0 ? *this : *this / mag;
}

Vec3& Vec3::normalize(){
	return *this = unitVector();
}

Vec3& Vec3::setMagnitude(float magnitude){
	if(nonzero()){
		normalize();
		*this *= magnitude;
	}

	return *this;
}

bool Vec3::equalsWithinThreshold(const Vec3& a, float threshold) const{
	return
		abs(x_ - a[0]) <= threshold &&
		abs(y_ - a[1]) <= threshold &&
		abs(z_ - a[2]) <= threshold;
}

void Vec3::setX(float x){
	x_ = x;
}

void Vec3::setY(float y){
	y_ = y;
}

void Vec3::setZ(float z){
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
