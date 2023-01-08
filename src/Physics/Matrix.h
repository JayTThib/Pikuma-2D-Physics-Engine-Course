#ifndef MATRIX_H
#define MATRIX_H

#include "VecN.h"

struct Matrix {
	int rowCount;
	int columnCount;
	VecN* rows;//The rows of the matrix with N columns inside

	Matrix();
	Matrix(int rowCount, int columnCount);
	Matrix(const Matrix& mat);
	~Matrix();

	void Zero();
	Matrix Transpose() const;

	const Matrix& operator = (const Matrix& mat);	//mat1 = mat2
	VecN operator * (const VecN& vecN) const;		//mat1 * vecN
	Matrix operator * (const Matrix& mat) const;	//mat1 * mat2

	static VecN SolveGaussSeidel(const Matrix& A, const VecN& b);
};

#endif