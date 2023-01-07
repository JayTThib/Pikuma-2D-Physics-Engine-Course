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
	//todo
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
	//todo
}

VecN Matrix::operator * (const VecN& vecN) const {
	//todo
}