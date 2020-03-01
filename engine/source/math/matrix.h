#pragma once

/*
 *	matrix.h
 *
 *	Matrix class and functions.
 *
 */

#include<glad/glad.h>
#include<vector>
#include"vec3.h"
#include"quaternion.h"

using std::vector;

typedef vector<float> mat;

class Matrix{
	mat values_;
	int rows_;
	int cols_;

public:
	// Initialize to 4D identity matrix
	Matrix();

	// Initialize to all zeros or identity matrix
	Matrix(const int& rows, const int& cols, const bool& identity = false);
	
	// Initialize to given values
	Matrix(mat values, const int& rows, const int& cols);

	// Initialize to 4D projection matrix
	Matrix(const float& fovy, const float& aspect, const float& zNear, const float& zFar);


	friend Matrix operator+(const Matrix& a, const Matrix& b);
	friend Matrix operator-(const Matrix& a, const Matrix& b);
	friend Matrix operator*(const Matrix& a, const Matrix& b);
	friend Vec3 operator*(const Matrix& a, const Vec3& b);
	friend Matrix operator*(const Matrix& a, const float& b);
	friend Matrix operator/(const Matrix& a, const float& b);

	Matrix& operator+=(const Matrix& a);
	Matrix& operator-=(const Matrix& a);
	Matrix& operator*=(const Matrix& a);
	Matrix& operator*=(const float& a);
	Matrix& operator/=(const float& a);


	Matrix& translate(const float& x, const float& y, const float& z);
	Matrix& translate(const Vec3& v);

	Matrix& scale(const float& x, const float& y, const float& z);
	Matrix& scale(const Vec3& v);

	Matrix& rotate(const Quaternion& q);
	Matrix& rotate(Vec3 axis, const float& ang);
	Matrix& rotate(const float& x, const float& y, const float& z);
	Matrix& rotate(const Vec3& v);


	Matrix& place(const int& row, const int& col, const Matrix& a);
	Matrix& place(const int& row, const int& col, const Vec3& a, const bool& asRowVector = false);
	Matrix getSubMatrix(const int& row, const int& col, const int& numRows, const int& numCols) const;
	void swapRows(const int& row1, const int& row2);

	void transpose();
	Matrix getTranspose() const;

	Matrix getInverse() const;


	mat getValues() const;
	const float* getValuesPtr() const;

	float get(const int& row, const int& col) const;
	void set(const int& row, const int& col, float value);

	int getNumRows() const;
	int getNumCols() const;
};
