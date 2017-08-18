/*
 * Matrix.h
 *
 *  Created on: 11 Aug 2017
 *      Author: Sokol
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <vector>
using namespace std;

template<typename T>
class Matrix {
private:
	vector<vector<T> > mat;
	int rows;
	int cols;
public:
	Matrix(int myRows, int myCols, const T& myInitial);
	Matrix();

	Matrix(const Matrix<T>& rhs);
	virtual ~Matrix();

	// Matrix operations
	Matrix<T> transpose();
	Matrix<T>& inverse();
	Matrix<T> minor(const int row, const int col);
	double determinant();

	bool sameDimentions(const Matrix<T>& lhs, const Matrix<T>& rhs);

	Matrix<T>& operator=(const Matrix<T>& rhs);

	Matrix<T> operator+(const Matrix<T>& rhs);
	Matrix<T>& operator+=(const Matrix<T>& rhs);
	Matrix<T> operator-(const Matrix<T>& rhs);
	Matrix<T>& operator-=(const Matrix<T>& rhs);
	Matrix<T> operator*(const Matrix<T>& rhs);
	Matrix<T>& operator*=(const Matrix<T>& rhs);

	// matrix/scalar operations
	Matrix<T> operator+(const T& rhs);
	Matrix<T> operator-(const T& rhs);
	Matrix<T> operator*(const T& rhs);
	Matrix<T> operator/(const T& rhs);

	Matrix<T>& operator+=(const T& rhs);
	Matrix<T>& operator-=(const T& rhs);
	Matrix<T>& operator*=(const T& rhs);
	Matrix<T>& operator/=(const T& rhs);

	//Matrix/vector operations
	vector<T> operator*(const vector<T>& rhs);
	vector<T> diag();

	// Access individual elements
	T& operator()(const int& row, const int& col);
	T& operator()(const int& row);
	const T& operator()(const int& row, const int& col) const; // read/write
	const T& operator()(const int& row) const; // read/write

	bool hasElement(const int& row, const int& col);
	T& at(const int& row, const int& col);
	const T& at(const int& row, const int& col) const;

	Matrix<T>& fill(const T& myValue);
	void print() const;

	// Get dimensions of matrix
	int getRows() const;
	int getCols() const;

private:
	Matrix<T>& fillCheckBoard(const T& myValue1, const T& myValue2);
};

// extra functions
template<typename T>
Matrix<T> identity(const int& rows, const int& cols);

//template<typename T>
//Matrix<T> R1(const double& angleRadians);

template<typename T>
void printVector(const vector<T>& v);

template<typename T>
Matrix<double> cross(const Matrix<T>& u, const Matrix<T>& v);

template<typename T>
double norm(const Matrix<T>& r);

template<typename T>
double dot(const Matrix<T>& v1, const Matrix<T>& v2);

template<typename T>
Matrix<double> R3(const double& angleRadians);

#include "Matrix.cpp" // Necessary for template classes

#endif /* MATRIX_H_ */
