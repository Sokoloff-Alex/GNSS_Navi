#ifndef MATRIX_CPP_
#define MATRIX_CPP_

/*
 * Matrix.cpp
 *
 *  Created on: 11 Aug 2017
 *      Author: Sokol
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>
#include "Matrix.h"

using namespace std;

template<typename T>
Matrix<T>::Matrix(int myRows, int myCols, const T& myInitial) {
	mat.resize(myRows);
	for (int i = 0; i < myRows; ++i) {
		mat[i].resize(myCols, myInitial);
	}
	rows = myRows;
	cols = myCols;
}

template<typename T>
Matrix<T>::Matrix() { // defined for state vectors with 6 elements
	rows = 6;
	cols = 1;
	mat.resize(rows);
	for (int i = 0; i < rows; ++i) {
		mat[i].resize(1, 0.0);
	}
}

template<typename T>
Matrix<T>::Matrix(const Matrix<T>& rhs) {
	mat = rhs.mat;
	rows = rhs.getRows();
	cols = rhs.getCols();
}

template<typename T>
Matrix<T>::~Matrix() {
}

template<typename T>
bool Matrix<T>::sameDimentions(const Matrix<T>& lhs, const Matrix<T>& rhs) {
	if (lhs.cols == rhs.cols && lhs.rows == rhs.rows) {
		return true;
	} else {
		return false;
	}
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& rhs) {
	if (this == &rhs) {
		return *this;
	}
	if (sameDimentions(*this, rhs)) {
		mat = rhs.mat;
		return *this;
	} else {
		cout << "Matrixes have different sizes!" << endl;
		throw "Matrixes have different sizes! ( operator= )";
	}
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& rhs) {
	if (!sameDimentions(*this, rhs)) {
		throw "Matrices have different dimensions";
	} else {
		Matrix result(rows, cols, 0.0);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				result(i, j) = this->mat[i][j] + rhs(i, j);
			}
		}
		return result;
	}
}

template<typename T>
Matrix<T>& Matrix<T>::operator+=(const Matrix<T>& rhs) {
	if (!sameDimentions(*this, rhs)) {
		throw "Matrices have different dimensions";
	} else {
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				this->mat[i][j] += rhs(i, j);
			}
		}
		return *this;
	}
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& rhs) {
	if (!sameDimentions(*this, rhs)) {
		throw "Matrices have different dimensions";
	} else {
		Matrix result(rows, cols, 0.0);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				result(i, j) = this->mat[i][j] - rhs(i, j);
			}
		}
		return result;
	}
}

template<typename T>
Matrix<T>& Matrix<T>::operator-=(const Matrix<T>& rhs) {
	if (!sameDimentions(*this, rhs)) {
		throw "Matrices have different dimensions";
	} else {
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				this->mat[i][j] -= rhs(i, j);
			}
		}
		return *this;
	}
}

// multiplication row by column (inner product or dot product)
// A[n,m] * B[m,k] = C[n,k];
template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& rhs) {
	if (cols != rhs.rows) {
		throw "Matrices have different dimensions";
	} else {
		Matrix result(rows, rhs.cols, 0.0);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < rhs.cols; ++j) {
				for (int k = 0; k < cols; ++k) {
					result(i, j) += this->mat[i][k] * rhs(k, j); // ok
				}
			}
		}
		return result;
	}
}

// matrix/scalar operations
template<typename T>
Matrix<T> Matrix<T>::operator+(const T& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] + rhs;
		}
	}
	return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator+=(const T& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] + rhs;
		}
	}
	return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const T& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] - rhs;
		}
	}
	return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator-=(const T& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] - rhs;
		}
	}
	return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const T& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] * rhs;
		}
	}
	return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator*=(const T& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] * rhs;
		}
	}
	return *this;
}

template<typename T>
Matrix<T> Matrix<T>::operator/(const T& rhs) {
	if (rhs == 0) {
		cout << "Error, Division by zero in matrix operations" << endl;
		throw "Division by zero in matrix operations";
	}
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] / rhs;
		}
	}
	return result;
}

template<typename T>
Matrix<T>& Matrix<T>::operator/=(const T& rhs) {
	if (rhs == 0) {
		cout << "Error, Division by zero in matrix operations" << endl;
		throw "Division by zero in matrix operations";
	}
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] / rhs;
		}
	}
	return *this;
}

// Matrix/vector multiplication
template<typename T>
vector<T> Matrix<T>::operator*(const vector<T>& rhs) {
	if (cols != rhs.size()) {
		throw "Error, matrix*vector multiplication, wrong size";
	}
	vector<T> result(rows, 0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result[i] = this->mat[i][j] * rhs[j]; // ok
		}
	}
	return result;
}

// get diagonal elements
template<typename T>
vector<T> Matrix<T>::diag() {
	int shortest_dim = 0;
	if (rows < cols) {
		shortest_dim = rows;
	} else {
		shortest_dim = rows;
	}
	vector<T> diagonal(shortest_dim, 0);
	for (int i = 0; i < shortest_dim; ++i) {
		diagonal[i] = this->mat[i][i];
	}
	return diagonal;
}

template<typename T>
T& Matrix<T>::operator()(const int& row, const int& col) {
	return this->mat[row][col];
}

template<typename T>
T& Matrix<T>::operator()(const int& row) {
	int col = 0;
	return this->mat[row][col];
}

template<typename T>
const T& Matrix<T>::operator()(const int& row, const int& col) const {
	return this->mat[row][col];
}

template<typename T>
const T& Matrix<T>::operator()(const int& row) const {
	int col = 0;
	return this->mat[row][col];
}

template<typename T>
bool Matrix<T>::hasElement(const int& row, const int& col) {
	if ((0 <= row && row < rows) && (0 <= col && col < cols)) {
		return true;
	} else {
		return false;
	}
}

template<typename T>
T& Matrix<T>::at(const int& row, const int& col) {
	return this->mat[row][col];
}

template<typename T>
const T& Matrix<T>::at(const int& row, const int& col) const {
	return this->mat[row][col];
}

template<typename T>
int Matrix<T>::getRows() const {
	return rows;
}

template<typename T>
int Matrix<T>::getCols() const {
	return cols;
}

template<typename T>
Matrix<T>& Matrix<T>::fill(const T& myValue) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = myValue;
		}
	}
	return *this;
}

template<typename T>
void Matrix<T>::print() const {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			cout << fixed << setw(20) << setprecision(6) << this->mat[i][j] << " ";
		}
		cout << endl;
	}
}

template<typename T>
Matrix<T> Matrix<T>::transpose() {
	Matrix result(cols, rows, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(j, i) = mat[i][j];
		}
	}
	return result;
}

template<typename T>
Matrix<T> Matrix<T>::minor(const int row, const int col) {
	Matrix minor(rows - 1, cols - 1, 0.0);

	for (int i = 0; i < row; ++i) {
		for (int j = 0; j < col; ++j) {
			minor(i, j) = this->mat[i][j];
		}
	}
	for (int i = 0; i < row; ++i) {
		for (int j = col + 1; j < cols; ++j) {
			minor(i, j - 1) = this->mat[i][j];
		}
	}
	for (int i = row + 1; i < rows; ++i) {
		for (int j = 0; j < col; ++j) {
			minor(i - 1, j) = this->mat[i][j];
		}
	}
	for (int i = row + 1; i < rows; ++i) {
		for (int j = col + 1; j < cols; ++j) {
			minor(i - 1, j - 1) = this->mat[i][j];
		}
	}
	return minor;
}

template<typename T>
double Matrix<T>::determinant() {
	double determinant = 0; // use always double
	if (rows == cols) {
		if (rows == 1) {
			determinant = mat[0][0];
		}
		if (rows == 2) {
			determinant = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
		}
		if (rows == 3) {
			determinant = mat[0][0] * mat[1][1] * mat[2][2] + mat[0][1] * mat[1][2] * mat[2][0]
					+ mat[0][2] * mat[1][0] * mat[2][1] - mat[0][2] * mat[1][1] * mat[2][0]
					- mat[0][1] * mat[1][0] * mat[2][2] - mat[0][0] * mat[1][2] * mat[2][1];
		}

		if (rows > 3) { // for larger dimensions do recursive
			Matrix Minor(rows - 1, cols - 1, 0.0);
			for (int i = 0; i < rows; ++i) {
				Minor = this->minor(0, i);
				int sign = 0;
				if (i % 2 == 0) {
					sign = 1;
				} else {
					sign = -1;
				}
				double det = sign * mat[0][i] * Minor.determinant();
				determinant += det;
			}
		}
	} else {
		cout << "Error, call for determinant for NON-square matrix" << endl;
	}
	return determinant;
}

template<typename T>
Matrix<T>& Matrix<T>::fillCheckBoard(const T& myValue1, const T& myValue2) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			if ((i + j) % 2 == 0) {
				this->mat[i][j] = myValue1;
			} else {
				this->mat[i][j] = myValue2;
			}
		}
	}
	return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::inverse() {
	// use Cramer law, ineffective for large matrixes
	if (rows != cols) {
		throw "Error, matrix is rectangular (matrix inversion)";
	}
	if (this->determinant() == 0) {
		throw "Zero determinant (matrix inversion)";
	}
	Matrix Cofactor(rows, cols, 0);
	Matrix Minor(rows - 1, cols - 1, 0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			Minor = this->minor(i, j);
			Cofactor(i, j) = Minor.determinant();
			if ((i + j) % 2 == 1) {
				Cofactor(i, j) *= -1;
			}
		}
	}
	Matrix adjugate = Cofactor.transpose();
	double determinant = this->determinant();
	*this = adjugate / determinant;
	return *this;
}

template<typename T>
Matrix<T> identity(const int& rows, const int& cols) {
	Matrix<T> IdentityMatrix(rows, cols, 0);
	int minDimension = 0;
	if (rows < cols) {
		minDimension = rows;
	} else {
		minDimension = cols;
	}
	for (int i = 0; i < minDimension; ++i) {
		IdentityMatrix(i, i) = 1;
	}
	return IdentityMatrix;
}

template<typename T>
void printVector(const vector<T>& v) {
	for (auto& item : v) {
		cout << item << endl;
	}
}

template<typename T>
Matrix<double> cross(const Matrix<T>& u, const Matrix<T>& v) {
	if (u.getRows() == 3 && v.getRows() == 3 && u.getCols() == 1 && v.getCols() == 1) {
		Matrix<double> res(3, 1, 0);
		res(0, 0) = u(1, 0) * v(2, 0) - u(2, 0) * v(1, 0);
		res(1, 0) = u(2, 0) * v(0, 0) - u(0, 0) * v(2, 0);
		res(2, 0) = u(0, 0) * v(1, 0) - u(1, 0) * v(1, 0);
		return res;
	}
	if (u.getRows() == 1 && v.getRows() == 1 && u.getCols() == 3 && v.getCols() == 3) {
		Matrix<double> res(1, 3, 0);
		res(0, 0) = u(0, 1) * v(0, 2) - u(0, 2) * v(0, 1);
		res(0, 1) = u(0, 2) * v(0, 0) - u(0, 0) * v(0, 2);
		res(0, 2) = u(0, 0) * v(0, 1) - u(0, 1) * v(0, 1);
		return res;
	} else {
		cout << "Error: wrong dimensions!" << endl;
		throw "Error: wrong dimensions!";
	}
}

template<typename T>
double norm(const Matrix<T>& r) {
	double x = r(0, 0);
	double y = r(1, 0);
	double z = r(2, 0);
	double mag = sqrt(x * x + y * y + z * z);
	return mag;
}

template<typename T>
double dot(const Matrix<T>& v1, const Matrix<T>& v2) {
	if (v1.getCols() == v2.getRows() && v1.getRows() == 1 && v2.getCols() == 1) {
		double res = 0;
		for (int i = 0; i < v1.getCols(); ++i) {
			res += v1(0, i) * v2(i, 0);
		}
		return res;
	} else {
		cout << "Error: wrong dimensions!" << endl;
		throw "Error: wrong dimensions!";
	}
}

template<typename T>
Matrix<double> R3(const double& angleRadians) {
	Matrix<double> R3(3, 3, 0.0);
	R3(0, 0) = cos(angleRadians);
	R3(0, 1) = sin(angleRadians);
	R3(1, 0) = -sin(angleRadians);
	R3(1, 1) = cos(angleRadians);
	R3(2, 2) = 1;
	return R3;
}

#endif

