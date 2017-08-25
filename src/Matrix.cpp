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

Matrix::Matrix(int myRows, int myCols, const double& myInitial) {
	mat.resize(myRows);
	for (int i = 0; i < myRows; ++i) {
		mat[i].resize(myCols, myInitial);
	}
	rows = myRows;
	cols = myCols;
}

Matrix::Matrix() { // defined for state vectors with 6 elements
	rows = 6;
	cols = 1;
	mat.resize(rows);
	for (int i = 0; i < rows; ++i) {
		mat[i].resize(1, 0.0);
	}
}

Matrix::Matrix(const Matrix& rhs) {
	mat = rhs.mat;
	rows = rhs.getRows();
	cols = rhs.getCols();
}

Matrix::~Matrix() {
}

bool Matrix::sameDimentions(const Matrix& lhs, const Matrix& rhs) {
	if (lhs.cols == rhs.cols && lhs.rows == rhs.rows) {
		return true;
	} else {
		return false;
	}
}

bool Matrix::sameDimentions(const Matrix& lhs, const Matrix& rhs) const {
	if (lhs.cols == rhs.cols && lhs.rows == rhs.rows) {
		return true;
	} else {
		return false;
	}
}

Matrix& Matrix::operator=(const Matrix& rhs) {
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

Matrix Matrix::operator+(const Matrix& rhs) {
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

Matrix& Matrix::operator+=(const Matrix& rhs) {
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

Matrix Matrix::operator-(const Matrix& rhs) {
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

Matrix Matrix::operator-(const Matrix& rhs) const {
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

Matrix& Matrix::operator-=(const Matrix& rhs) {
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

Matrix Matrix::operator*(const Matrix& rhs) {
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

Matrix Matrix::operator+(const double& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] + rhs;
		}
	}
	return result;
}

Matrix& Matrix::operator+=(const double& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] + rhs;
		}
	}
	return *this;
}

Matrix Matrix::operator-(const double& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] - rhs;
		}
	}
	return result;
}

Matrix& Matrix::operator-=(const double& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] - rhs;
		}
	}
	return *this;
}

Matrix Matrix::operator*(const double& rhs) {
	Matrix result(rows, cols, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(i, j) = this->mat[i][j] * rhs;
		}
	}
	return result;
}

Matrix& Matrix::operator*=(const double& rhs) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = this->mat[i][j] * rhs;
		}
	}
	return *this;
}

Matrix Matrix::operator/(const double& rhs) {
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

Matrix& Matrix::operator/=(const double& rhs) {
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

vector<double> Matrix::operator*(const vector<double>& rhs) {
	if (cols != rhs.size()) {
		throw "Error, matrix*vector multiplication, wrong size";
	}
	vector<double> result(rows, 0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result[i] = this->mat[i][j] * rhs[j]; // ok
		}
	}
	return result;
}

// get diagonal elements

vector<double> Matrix::diag() {
	int shortest_dim = 0;
	if (rows < cols) {
		shortest_dim = rows;
	} else {
		shortest_dim = rows;
	}
	vector<double> diagonal(shortest_dim, 0);
	for (int i = 0; i < shortest_dim; ++i) {
		diagonal[i] = this->mat[i][i];
	}
	return diagonal;
}

double& Matrix::operator()(const int& row, const int& col) {
	return this->mat[row][col];
}

double& Matrix::operator()(const int& row) {
	int col = 0;
	return this->mat[row][col];
}

const double& Matrix::operator()(const int& row, const int& col) const {
	return this->mat[row][col];
}

const double& Matrix::operator()(const int& row) const {
	int col = 0;
	return this->mat[row][col];
}

bool Matrix::hasElement(const int& row, const int& col) {
	if ((0 <= row && row < rows) && (0 <= col && col < cols)) {
		return true;
	} else {
		return false;
	}
}

double& Matrix::at(const int& row, const int& col) {
	return this->mat[row][col];
}

const double& Matrix::at(const int& row, const int& col) const {
	return this->mat[row][col];
}

int Matrix::getRows() const {
	return rows;
}

int Matrix::getCols() const {
	return cols;
}

Matrix& Matrix::fill(const double& myValue) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			this->mat[i][j] = myValue;
		}
	}
	return *this;
}

void Matrix::print() const {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			cout << fixed << setw(20) << setprecision(6) << this->mat[i][j]
					<< " ";
		}
		cout << endl;
	}
}

Matrix Matrix::transpose() {
	Matrix result(cols, rows, 0.0);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result(j, i) = mat[i][j];
		}
	}
	return result;
}

Matrix Matrix::minor(const int row, const int col) {
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

double Matrix::determinant() {
	double determinant = 0; // use always double
	if (rows == cols) {
		if (rows == 1) {
			determinant = mat[0][0];
		}
		if (rows == 2) {
			determinant = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
		}
		if (rows == 3) {
			determinant = mat[0][0] * mat[1][1] * mat[2][2]
					+ mat[0][1] * mat[1][2] * mat[2][0]
					+ mat[0][2] * mat[1][0] * mat[2][1]
					- mat[0][2] * mat[1][1] * mat[2][0]
					- mat[0][1] * mat[1][0] * mat[2][2]
					- mat[0][0] * mat[1][2] * mat[2][1];
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

Matrix& Matrix::fillCheckBoard(const double& myValue1, const double& myValue2) {
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

Matrix& Matrix::inverse() {
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

Matrix identity(const int& rows, const int& cols) {
	Matrix IdentityMatrix(rows, cols, 0);
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

void printVector(const vector<double>& v) {
	for (auto& item : v) {
		cout << item << endl;
	}
}

Matrix cross(const Matrix& u, const Matrix& v) {
	if (u.getRows() == 3 && v.getRows() == 3 && u.getCols() == 1
			&& v.getCols() == 1) {
		Matrix res(3, 1, 0);
		res(0, 0) = u(1, 0) * v(2, 0) - u(2, 0) * v(1, 0);
		res(1, 0) = u(2, 0) * v(0, 0) - u(0, 0) * v(2, 0);
		res(2, 0) = u(0, 0) * v(1, 0) - u(1, 0) * v(1, 0);
		return res;
	}
	if (u.getRows() == 1 && v.getRows() == 1 && u.getCols() == 3
			&& v.getCols() == 3) {
		Matrix res(1, 3, 0);
		res(0, 0) = u(0, 1) * v(0, 2) - u(0, 2) * v(0, 1);
		res(0, 1) = u(0, 2) * v(0, 0) - u(0, 0) * v(0, 2);
		res(0, 2) = u(0, 0) * v(0, 1) - u(0, 1) * v(0, 1);
		return res;
	} else {
		cout << "Error: wrong dimensions!" << endl;
		throw "Error: wrong dimensions!";
	}
}

double norm(const Matrix& r) {
	double mag = 0;
	if (r.getRows() == 3) {
		double x = r(0, 0);
		double y = r(1, 0);
		double z = r(2, 0);
		mag = sqrt(x * x + y * y + z * z);
	}
	if (r.getRows() == 2) {
		double x = r(0, 0);
		double y = r(1, 0);
		mag = sqrt(x * x + y * y);
	}
	if (r.getRows() == 1) {
		mag = r(0, 0);
	}
	return mag;
}

double dot(const Matrix& v1, const Matrix& v2) {
	if (v1.getCols() == v2.getRows() && v1.getRows() == 1
			&& v2.getCols() == 1) {
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

Matrix R3(const double& angleRadians) {
	Matrix R_3(3, 3, 0.0);
	R_3(0, 0) = cos(angleRadians);
	R_3(0, 1) = sin(angleRadians);
	R_3(1, 0) = -sin(angleRadians);
	R_3(1, 1) = cos(angleRadians);
	R_3(2, 2) = 1;
	return R_3;
}

Matrix R2(const double& angleRadians) {
	Matrix R2(3, 3, 0.0);
	R2(0, 0) = cos(angleRadians);
	R2(0, 1) = -sin(angleRadians);
	R2(1, 1) = 1;
	R2(2, 0) = sin(angleRadians);
	R2(2, 2) = cos(angleRadians);
	return R2;
}

Matrix R1(const double& angleRadians) {
	Matrix R1(3, 3, 0.0);
	R1(0, 0) = 1;
	R1(1, 1) = cos(angleRadians);
	R1(1, 2) = sin(angleRadians);
	R1(2, 1) = -sin(angleRadians);
	R1(2, 2) = cos(angleRadians);
	return R1;
}

