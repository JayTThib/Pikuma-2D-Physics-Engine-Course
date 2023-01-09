#include "Matrix.h"
#include <cmath>
#include <iostream>

Matrix::Matrix(): rowCount(0), columnCount(0), rows(nullptr) {}

Matrix::Matrix(int rowCount, int columnCount): rowCount(rowCount), columnCount(columnCount) {
	rows = new VecN[rowCount];
	for (int i = 0; i < rowCount; i++) {
		rows[i] = VecN(columnCount);
	}
}

Matrix::Matrix(const Matrix& mat) {
	*this = mat;
}

Matrix::~Matrix() {
	delete[] rows;
}

void Matrix::Zero() {
	for (int i = 0; i < rowCount; i++) {
		rows[i].Zero();
	}
}

Matrix Matrix::Transpose() const {
	Matrix result(columnCount, rowCount);
	for (int i = 0; i < rowCount; i++) {
		for (int j = 0; j < columnCount; j++) {
			result.rows[j][i] = rows[i][j];
		}
	}
	return result;
}

const Matrix& Matrix::operator = (const Matrix& mat) {
	rowCount = mat.rowCount;
	columnCount = mat.columnCount;
	rows = new VecN[rowCount];

	for (int i = 0; i < rowCount; i++) {
		rows[i] = mat.rows[i];
	}
	return *this;
}

VecN Matrix::operator * (const VecN& vecN) const {
	if (vecN.componentNum != columnCount) {
		return vecN;
	}

	VecN result(rowCount);
	for (int i = 0; i < rowCount; i++) {
		result[i] = vecN.Dot(rows[i]);
	}
	return result;
}

Matrix Matrix::operator * (const Matrix& mat) const {
	if (mat.rowCount != columnCount && mat.columnCount != rowCount) {
		return mat;
	}

	Matrix transposed = mat.Transpose();
	Matrix result(rowCount, mat.columnCount);

	for (int i = 0; i < rowCount; i++) {
		for (int j = 0; j < mat.columnCount; j++) {
			result.rows[i][j] = rows[i].Dot(transposed.rows[j]);
		}
	}
	return result;
}

VecN Matrix::SolveGaussSeidel(const Matrix& mat, const VecN& vec) {
	VecN X(vec.componentNum);
	X.Zero();

	for (int j = 0; j < vec.componentNum; j++) {
		for (int i = 0; i < vec.componentNum; i++) {
			float deltaX = (vec[i] / mat.rows[i][i]) - (mat.rows[i].Dot(X) / mat.rows[i][i]);
			if (!isnan(deltaX)) {
				X[i] += deltaX;
			}
		}
	}
	
	return X;
}