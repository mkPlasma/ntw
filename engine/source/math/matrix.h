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
	Matrix(int rows, int cols, bool identity = false);
	
	// Initialize to given values
	Matrix(mat values, int rows, int cols);

	// Initialize to 4D projection matrix
	Matrix(float fovy, float aspect, float zNear, float zFar);


	friend Matrix operator+(const Matrix& a, const Matrix& b);
	friend Matrix operator-(const Matrix& a, const Matrix& b);
	friend Matrix operator*(const Matrix& a, const Matrix& b);
	friend Vec3 operator*(const Matrix& a, const Vec3& b);
	friend Matrix operator*(const Matrix& a, float b);
	friend Matrix operator/(const Matrix& a, float b);

	Matrix& operator+=(const Matrix& a);
	Matrix& operator-=(const Matrix& a);
	Matrix& operator*=(const Matrix& a);
	Matrix& operator*=(float a);
	Matrix& operator/=(float a);


	Matrix& translate(float x, float y, float z);
	Matrix& translate(const Vec3& v);

	Matrix& scale(float x, float y, float z);
	Matrix& scale(const Vec3& v);

	Matrix& rotate(const Quaternion& q);
	Matrix& rotate(Vec3 axis, float ang);
	Matrix& rotate(float x, float y, float z);
	Matrix& rotate(const Vec3& v);


	Matrix& place(int row, int col, const Matrix& a);
	Matrix& place(int row, int col, const Vec3& a, bool asRowVector = false);
	Matrix getSubMatrix(int row, int col, int numRows, int numCols) const;
	void swapRows(int row1, int row2);

	void transpose();
	Matrix getTranspose() const;

	Matrix getInverse() const;


	mat getValues() const;
	const float* getValuesPtr() const;

	float get(int row, int col) const;
	void set(int row, int col, float value);

	int getNumRows() const;
	int getNumCols() const;
};
