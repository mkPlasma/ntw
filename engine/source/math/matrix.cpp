#include"matrix.h"

#include"mathFunc.h"
#include<algorithm>
#include<stdexcept>
#include<string>

using ntw::toRadians;
using std::max;
using std::min;


// Initialize to 4D identity matrix
Matrix::Matrix() : rows_(4), cols_(4) {
	values_ = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
}

// Initialize to all zeros or identity matrix
Matrix::Matrix(int rows, int cols, bool identity) : rows_(rows), cols_(cols) {
	values_ = mat((size_t)rows * cols);

	if(identity){
		int num = min(rows, cols);

		for(int i = 0; i < num; i++)
			set(i, i, 1);
	}
}

// Initialize to given values
Matrix::Matrix(mat values, int rows, int cols) : values_(values), rows_(rows), cols_(cols) {

	// Check value list is correctly sized
	if(values_.size() != (size_t)rows * cols)
		throw std::runtime_error("Matrix has incorrect size: given " +
		std::to_string(values_.size()) + ", expected " +
		std::to_string(rows * cols));
}

// Initialize to 4D projection matrix
Matrix Matrix::projectionMatrix(float fovy, float aspect, float zNear, float zFar){

	float f = 1 / tanf(toRadians(fovy) / 2);
	float zr = zNear - zFar;

	return Matrix(
		{
			f / aspect,	0,	0,	0,
			0,			f,	0,	0,
			0,			0,	(zFar + zNear) / zr,		-1,
			0,			0,	(2 * zFar * zNear) / zr,	0
		}, 4, 4);
}

// 4D projection matrix with oblique near clipping plane
Matrix Matrix::projectionMatrix(float fovy, float aspect, float zFar, const Vec3& normal, float dist){

	Matrix proj = projectionMatrix(fovy, aspect, dist, zFar);

	auto l_sign = [](float val) -> int {
		return val == 0 ? 0 : val > 0 ? 1 : -1;
	};

	float cx = normal[0];
	float cy = normal[1];
	float cz = normal[2];
	float cw = dist;

	float qx = (l_sign(cx) + proj.get(2, 0)) / proj.get(0, 0);
	float qy = (l_sign(cy) + proj.get(2, 1)) / proj.get(1, 1);
	float qz = -1;
	float qw = 1 / zFar;

	float fac = 2 / ((cx * qx) + (cy * qy) + (cz * qz) + (cw * qw));

	proj.set(0, 2, fac * cx);
	proj.set(1, 2, fac * cy);
	proj.set(2, 2, fac * cz + 1);
	proj.set(3, 2, fac * cw);

	return proj;
}

Matrix operator+(const Matrix& a, const Matrix& b){

	// Resulting matrix size
	int rows = max(a.getNumRows(), b.getNumRows());
	int cols = max(a.getNumCols(), b.getNumCols());

	Matrix result(rows, cols);

	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			result.set(i, j, a.get(i, j) + b.get(i, j));

	return result;
}

Matrix operator-(const Matrix& a, const Matrix& b){

	// Resulting matrix size
	int rows = max(a.getNumRows(), b.getNumRows());
	int cols = max(a.getNumCols(), b.getNumCols());

	Matrix result(rows, cols);

	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			result.set(i, j, a.get(i, j) - b.get(i, j));

	return result;
}

Matrix operator*(const Matrix& a, const Matrix& b){
	
	int m = a.getNumRows();	// a rows
	int n = a.getNumCols();	// shared dimension
	int o = b.getNumCols();	// b cols

	// Check dimensions
	if(n != b.getNumRows())
		throw std::runtime_error("Matrix multiplication dimension mismatch: " +
			std::to_string(n) + ", " +
			std::to_string(b.getNumRows()));


	Matrix result(m, o);

	for(int i = 0; i < m; i++)	// rows
		for(int j = 0; j < o; j++)	// cols
			for(int k = 0; k < n; k++)	// shared
				result.set(i, j, result.get(i, j) + a.get(i, k) * b.get(k, j));

	return result;
}

Vec3 operator*(const Matrix& a, const Vec3& b){
	if(a.getNumCols() == 4)
		return Vec3(
			a.get(0, 0) * b[0] + a.get(0, 1) * b[1] + a.get(0, 2) * b[2] + a.get(0, 3),
			a.get(1, 0) * b[0] + a.get(1, 1) * b[1] + a.get(1, 2) * b[2] + a.get(1, 3),
			a.get(2, 0) * b[0] + a.get(2, 1) * b[1] + a.get(2, 2) * b[2] + a.get(2, 3)
		);

	return Vec3(
		a.get(0, 0) * b[0] + a.get(0, 1) * b[1] + a.get(0, 2) * b[2],
		a.get(1, 0) * b[0] + a.get(1, 1) * b[1] + a.get(1, 2) * b[2],
		a.get(2, 0) * b[0] + a.get(2, 1) * b[1] + a.get(2, 2) * b[2]
	);
}

Matrix operator*(const Matrix& a, float b){

	int rows = a.getNumRows();
	int cols = a.getNumCols();

	Matrix result(rows, cols);

	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			result.set(i, j, a.get(i, j) * b);

	return result;
}

Matrix operator/(const Matrix& a, float b){

	int rows = a.getNumRows();
	int cols = a.getNumCols();

	Matrix result(rows, cols);

	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			result.set(i, j, a.get(i, j) / b);

	return result;
}

bool operator==(const Matrix& a, const Matrix& b){

	if(a.getNumRows() != b.getNumRows() || a.getNumCols() != b.getNumCols())
		return false;

	int rows = a.getNumRows();
	int cols = a.getNumCols();

	for(int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++)
			if(a.get(i, j) != b.get(i, j))
				return false;

	return true;
}

bool operator!=(const Matrix& a, const Matrix& b){
	return !(a == b);
}

Matrix& Matrix::operator+=(const Matrix& a){
	return *this = *this + a;
}

Matrix& Matrix::operator-=(const Matrix& a){
	return *this = *this - a;
}

Matrix& Matrix::operator*=(const Matrix& a){
	return *this = *this * a;
}

Matrix& Matrix::operator*=(float a){
	return *this = *this * a;
}

Matrix& Matrix::operator/=(float a){
	return *this = *this / a;
}


Matrix& Matrix::translate(float x, float y, float z){
	Matrix t;
	t.set(0, 3, x);
	t.set(1, 3, y);
	t.set(2, 3, z);

	return *this = t * *this;
}

Matrix& Matrix::translate(const Vec3& v){
	return translate(v[0], v[1], v[2]);
}

Matrix& Matrix::scale(float x, float y, float z){
	Matrix t = rows_ == 4 ? Matrix() : Matrix(3, 3, true);
	t.set(0, 0, x);
	t.set(1, 1, y);
	t.set(2, 2, z);

	return *this = t * *this;
}

Matrix& Matrix::scale(const Vec3& v){
	return scale(v[0], v[1], v[2]);
}

Matrix& Matrix::rotate(const Quaternion& q){

	float x = q[0];
	float y = q[1];
	float z = q[2];
	float w = q[3];
	float x2 = 2 * powf(x, 2);
	float y2 = 2 * powf(y, 2);
	float z2 = 2 * powf(z, 2);

	Matrix t = rows_ == 4 ? Matrix() : Matrix(3, 3, true);
	t.set(0, 0, 1 - y2 - z2);
	t.set(0, 1, 2 * (x * y - w * z));
	t.set(0, 2, 2 * (x * z + w * y));

	t.set(1, 0, 2 * (x * y + w * z));
	t.set(1, 1, 1 - x2 - z2);
	t.set(1, 2, 2 * (y * z - w * x));

	t.set(2, 0, 2 * (x * z - w * y));
	t.set(2, 1, 2 * (y * z + w * x));
	t.set(2, 2, 1 - x2 - y2);

	return *this = (rows_ == 4 ? t : t.getSubMatrix(0, 0, 3, 3)) * *this;
}

Matrix& Matrix::rotate(Vec3 axis, float ang, bool degrees){

	ang = degrees ? toRadians(ang) : ang;

	float c = cosf(ang);
	float s = sinf(ang);

	axis.normalize();
	Vec3 tmp = (1 - c) * axis;

	Matrix t = rows_ == 4 ? Matrix() : Matrix(3, 3, true);
	t.set(0, 0, c + tmp[0] * axis[0]);
	t.set(0, 1, tmp[0] * axis[1] + s * axis[2]);
	t.set(0, 2, tmp[0] * axis[2] - s * axis[1]);

	t.set(1, 0, tmp[1] * axis[0] - s * axis[2]);
	t.set(1, 1, c + tmp[1] * axis[1]);
	t.set(1, 2, tmp[1] * axis[2] + s * axis[0]);

	t.set(2, 0, tmp[2] * axis[0] + s * axis[1]);
	t.set(2, 1, tmp[2] * axis[1] - s * axis[0]);
	t.set(2, 2, c + tmp[2] * axis[2]);

	return *this = (rows_ == 4 ? t : t.getSubMatrix(0, 0, 3, 3)) * *this;
}

Matrix& Matrix::rotate(float x, float y, float z, bool degrees){

	float ax = degrees ? toRadians(x) : x;
	float ay = degrees ? toRadians(y) : y;
	float az = degrees ? toRadians(z) : z;

	float cx = cosf(ax);
	float sx = sinf(ax);

	float cy = cosf(ay);
	float sy = sinf(ay);

	float cz = cosf(az);
	float sz = sinf(az);

	Matrix t = Matrix();
	t.set(0, 0, cy * cz);
	t.set(0, 1, (-cx * sz) + (sx * sy * cz));
	t.set(0, 2, (sx * sz) + (cx * sy * cz));

	t.set(1, 0, cy * sz);
	t.set(1, 1, (cx * cz) + (sx * sy * sz));
	t.set(1, 2, (-sx * cz) + (cx * sx * sz));

	t.set(2, 0, -sy);
	t.set(2, 1, sx * cy);
	t.set(2, 2, cx * cy);

	return *this = (rows_ == 4 ? t : t.getSubMatrix(0, 0, 3, 3)) * *this;
}

Matrix& Matrix::rotate(const Vec3& v, bool reverse, bool degrees){

	if(!reverse){
		rotate(v[0], 0, 0, degrees);
		rotate(0, v[1], 0, degrees);
		rotate(0, 0, v[2], degrees);
	}
	else{
		rotate(0, 0, v[2], degrees);
		rotate(0, v[1], 0, degrees);
		rotate(v[0], 0, 0, degrees);
	}

	return *this;
}


Matrix& Matrix::place(int row, int col, const Matrix& a){

	int cols = a.getNumCols();
	int rows = a.getNumRows();

	for(int r = 0; r < rows; r++)
		for(int c = 0; c < cols; c++)
			set(row + r, col + c, a.get(r, c));

	return *this;
}

Matrix& Matrix::place(int row, int col, const Vec3& a, bool asRowVector){

	for(int i = 0; i < 3; i++)
		set(row + (!asRowVector ? i : 0), col + (asRowVector ? i : 0), a[i]);

	return *this;
}

void Matrix::swapRows(int row1, int row2){

	if(row1 == row2)
		return;

	for(int c = 0; c < cols_; c++){
		float tmp = get(row1, c);
		set(row1, c, get(row2, c));
		set(row2, c, tmp);
	}
}

Matrix Matrix::getSubMatrix(int row, int col, int numRows, int numCols) const{
	mat values;

	for(int r = 0; r < numRows; r++)
		for(int c = 0; c < numCols; c++)
			values.push_back(get(row + r, col + c));

	return Matrix(values, numRows, numCols);
}

void Matrix::transpose(){
	mat newValues;

	for(int c = 0; c < cols_; c++)
		for(int r = 0; r < rows_; r++)
			newValues.push_back(get(r, c));

	values_ = newValues;

	int tmp = rows_;
	rows_ = cols_;
	cols_ = tmp;
}

Matrix Matrix::getTranspose() const{

	mat values;

	for(int c = 0; c < cols_; c++)
		for(int r = 0; r < rows_; r++)
			values.push_back(get(r, c));

	return Matrix(values, cols_, rows_);
}

Matrix Matrix::getInverse() const{

	// Assume 3x3 matrix
	// Adjugate matrix
	Matrix adj = Matrix(rows_, cols_);

	for(int r = 0; r < rows_; r++){
		for(int c = 0; c < cols_; c++){
			// Get 2x2 determinant
			int r1 = r == 0 ? 1 : 0;
			int r2 = r == 2 ? 1 : 2;
			int c1 = c == 0 ? 1 : 0;
			int c2 = c == 2 ? 1 : 2;
			adj.set(r, c, get(r1, c1) * get(r2, c2) - get(r1, c2) * get(r2, c1));

			// Negate in a checkerboard pattern
			if((r * cols_ + c) % 2 == 1)
				adj.set(r, c, -adj.get(r, c));
		}
	}

	// Get 3x3 determinant
	float det = get(0, 0) * adj.get(0, 0) + get(0, 1) * adj.get(0, 1) + get(0, 2) * adj.get(0, 2);
	
	return adj.getTranspose() / det;


	/*
	// Create matrix containing current matrix and identity matrix
	Matrix identity = Matrix(rows_, cols_, true);
	Matrix m = Matrix(rows_, cols_ * 2);
	m.place(0, 0, *this);
	m.place(0, cols_, identity);

	// Get RREF to find inverse
	int r = 0;
	int c = 0;

	while(r < rows_ && c < cols_){

		// Get pivot in column
		float pivot = 0;
		int pivotRow = 0;

		for(int i = r; i < rows_; i++){
			float val = m.get(i, c);

			if(abs(val) > abs(pivot)){
				pivot = val;
				pivotRow = i;
			}
		}

		// No pivot in this column, move to next
		// In this case matrix is not invertible
		if(pivot == 0){
			c++;
			continue;
		}

		// Reduce
		m.swapRows(r, pivotRow);

		// Scale row
		for(int i = c; i < cols_ * 2; i++)
			m.set(r, i, m.get(r, i) / pivot);

		// For rows other than pivot
		for(int i = 0; i < rows_; i++){

			if(i == r)
				continue;

			// For rows above pivot, only subtract if the value in the pivot column is nonzero
			if(i < r && m.get(i, c) == 0)
				continue;

			// Get factor (denominator is one since row was scaled)
			float f = m.get(i, c);

			// Set element in pivot column to zero
			m.set(i, c, 0);

			// Subtract from each element in row
			for(int j = c + 1; j < cols_ * 2; j++)
				m.set(i, j, m.get(i, j) - m.get(r, j) * f);
		}

		r++;
		c++;
	}

	Matrix test = m.getSubMatrix(0, 0, rows_, cols_);

	// Return right half of matrix
	return m.getSubMatrix(0, cols_, rows_, cols_);
	*/
}


mat Matrix::getValues() const{
	return values_;
}

const float* Matrix::getValuesPtr() const{
	return values_.data();
}


float Matrix::get(int row, int col) const{
	return values_[(size_t)row*cols_ + col];
}

void Matrix::set(int row, int col, float value){
	values_[(size_t)row * cols_ + col] = value;
}

int Matrix::getNumRows() const{
	return rows_;
}

int Matrix::getNumCols() const{
	return cols_;
}
