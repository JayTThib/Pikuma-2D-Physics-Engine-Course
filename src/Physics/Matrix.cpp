#include "Matrix.h"

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
	columnCount = mat.rowCount;
	rows = new VecN[rowCount];

	for (int i = 0; i < rowCount; i++) {
		rows[i] = mat.rows[i];
	}
	return *this;
}

Matrix Matrix::operator * (const Matrix& mat) const {
	if (mat.rowCount != rowCount && mat.columnCount != columnCount) {
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

VecN Matrix::SolveGaussSeidel(const Matrix& A, const VecN& b) {
	VecN X(b.componentNum);
	X.Zero();

	for (int iterations = 0; iterations < b.componentNum; iterations++) {
		for (int i = 0; i < b.componentNum; i++) {
			float deltaX = (b[i] / A.rows[i][i]) - (A.rows[i].Dot(X) / A.rows[i][i]);
			if (deltaX == deltaX) {//Avoids 'Not a Number' values
				X[i] += deltaX;
			}
		}
	}
	
	return X;
}