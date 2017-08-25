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

class Matrix {
private:
	vector<vector<double> > mat;
	int rows;
	int cols;
public:
	Matrix(int myRows, int myCols, const double& myInitial);
	Matrix();

	Matrix(const Matrix& rhs);
	virtual ~Matrix();

	// Matrix operations
	Matrix transpose();
	Matrix& inverse();
	Matrix minor(const int row, const int col);
	double determinant();

	bool sameDimentions(const Matrix& lhs, const Matrix& rhs);
	bool sameDimentions(const Matrix& lhs, const Matrix& rhs) const;

	Matrix& operator=(const Matrix& rhs);

	Matrix operator+(const Matrix& rhs);
	Matrix& operator+=(const Matrix& rhs);
	Matrix operator-(const Matrix& rhs);
	Matrix operator-(const Matrix& rhs) const;
	Matrix& operator-=(const Matrix& rhs);
	Matrix operator*(const Matrix& rhs);
	Matrix& operator*=(const Matrix& rhs);

	// matrix/scalar operations
	Matrix operator+(const double& rhs);
	Matrix operator-(const double& rhs);
	Matrix operator*(const double& rhs);
	Matrix operator/(const double& rhs);

	Matrix& operator+=(const double& rhs);
	Matrix& operator-=(const double& rhs);
	Matrix& operator*=(const double& rhs);
	Matrix& operator/=(const double& rhs);

	//Matrix/vector operations
	vector<double> operator*(const vector<double>& rhs);
	vector<double> diag();

	// Access individual elements
	double& operator()(const int& row, const int& col);
	double& operator()(const int& row);
	const double& operator()(const int& row, const int& col) const; // read/write
	const double& operator()(const int& row) const; // read/write

	bool hasElement(const int& row, const int& col);
	double& at(const int& row, const int& col);
	const double& at(const int& row, const int& col) const;

	Matrix& fill(const double& myValue);
	void print() const;

	// Get dimensions of matrix
	int getRows() const;
	int getCols() const;

private:
	Matrix& fillCheckBoard(const double& myValue1, const double& myValue2);
};

// extra functions
Matrix identity(const int& rows, const int& cols);

void printVector(const vector<double>& v);

Matrix cross(const Matrix& u, const Matrix& v);

double norm(const Matrix& r);

double dot(const Matrix& v1, const Matrix& v2);

Matrix R3(const double& angleRadians);

Matrix R2(const double& angleRadians);

Matrix R1(const double& angleRadians);

#endif /* MATRIX_H_ */
