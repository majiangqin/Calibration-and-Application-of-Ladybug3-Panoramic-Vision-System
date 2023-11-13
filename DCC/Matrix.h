// Matrix.h: implementation of matrix algorithm

#ifndef MATRIX_H
#define MATRIX_H

#include <map>

#pragma once

#define TINY 1.0e-10			// A small number
#define NMAX 5000				// Maximum allowed number of function evaluation
#define PI 3.14159265358979323846

class Matrix;
class LinearEquations;
class ImagePoint;

using namespace std;

class SimplexVertex
{
public:
	SimplexVertex() : m_nDim(0), m_pVector(NULL) {};
	SimplexVertex(int dim);
	SimplexVertex(int dim, const double* pVector);
	SimplexVertex(const SimplexVertex& vertex);
	SimplexVertex(const Matrix& matrix);
	SimplexVertex(const ImagePoint& point);
	~SimplexVertex();

	int getDim() const { return m_nDim; };
	void setDim(int dim);

	double getEntry(int i) const;
	void setEntry(int i, double x);

	void getVector(double* pVector) const;

   SimplexVertex getSubSpace(int start, int end) const;
   void setSubSpace(int start, int end, const SimplexVertex& vertex);

	SimplexVertex& operator = (const SimplexVertex& vertex);
	SimplexVertex& operator += (const SimplexVertex& vertex);
	SimplexVertex& operator *= (double val);
	double& operator ()(int i);
	double operator ()(int i) const;
	friend SimplexVertex operator + (const SimplexVertex& vertex1, const SimplexVertex& vertex2);
	friend SimplexVertex operator - (const SimplexVertex& vertex1, const SimplexVertex& vertex2);
	friend SimplexVertex operator * (double val, const SimplexVertex& vertex);

private:
	int m_nDim;
	double* m_pVector;
};

class Matrix
{
public:
	Matrix() : m_row(0), m_col(0), m_data(NULL) {};
	Matrix(int m, int n);
	Matrix(int m, int n, const double* pData);
	Matrix(const Matrix& matrix);
	Matrix(const SimplexVertex& vertex);
	~Matrix();

	int getRow() const { return m_row; }
	int getCol() const { return m_col; }
	int size() const { return m_row*m_col; }
   void resize(int row, int col);

	bool isSquare() const { return m_row == m_col; }

	double getEntry(int m, int n) const;
	void setEntry(int m, int n, double val);

	SimplexVertex getRowVec(int row) const;
	void setRowVec(int row, const SimplexVertex& vv);
	SimplexVertex getColVec(int col) const;
	void setColVec(int col, const SimplexVertex& vv);

	Matrix& operator = (const Matrix& matrix);
	Matrix& operator += (const Matrix& matrix);
	Matrix& operator *= (double val);
	double& operator ()(int i, int j);
	double operator ()(int i, int j) const;
	friend Matrix operator + (const Matrix& matrix1, const Matrix& matrix2);
	friend Matrix operator - (const Matrix& matrix1, const Matrix& matrix2);
	friend Matrix operator * (const Matrix& matrix1, const Matrix& matrix2);
	friend Matrix operator * (double val, const Matrix& matrix);

	friend Matrix Transpose(const Matrix& matrix); // Transpose matrix
	friend double Det(const Matrix& matrix); // determinant algorithm
	friend Matrix Inv(const Matrix& matrix); // Invert matrix
   // Matrix's rank can be computed with a method in MathUtil::GetGeometryProperty
	friend LinearEquations;

protected:
	// The implementation of LU decomposition comes from P2.3 in <Numerical Recipes in C, Second Edition (1992)> - Doolittle Decomposition
	// indx: an output n-size vector that records the row permutation effected by the partial pivoting
	// sign: output as positive or negative depending on whether the number of row interchanges was even or odd respectively
	// returns: True if matrix is not singular, false otherwise.
	bool LU(int* indx, bool* sign = 0);

	void SVD();

	// Givens Transformation comes from P5.7 in <Numerical Analysis, Fourth Edition>
	// After Givens Transformation, matrix becomes a upper triangular matrix.
   // Also see <Least Squares Computations by Givens Transformations Without Square Roots, J. Inst. Maths Applies, 1973(12): 329~336> - Givens-Gentleman Transformation
	// mL: observation matrix
	// returns: True if matrix is not singular, false otherwise.
	bool Givens(Matrix& mL);

private:
	int m_row;
	int m_col;
	double* m_data;
};

class SparseMatrix
{
public:

protected:
	// The implementation of LU decomposition comes from P2.4 in <Numerical Recipes in C, Second Edition (1992)>
	// for Tridiagonal and Band diagonal system of equations
	void LU();

private:
	int m_row;
	int m_col;
	int m_nzPointN;
	map<int,double> m_data;
};

class LinearEquations
{
public:
	// Solve A*X = B for general matrix
	static Matrix solve(Matrix& mA, Matrix& mB);
   
	// Solve A*X = B for sparse matrix, A is n-order square, B is n-row vector
	//static SimplexVertex solve(SparseMatrix& mA, SimplexVertex& vB);

   
};

class MatrixUtil
{
public:
	static Matrix generateRotateMatrix(double fi, double omiga, double kapa);
   static void computeRotateAngle(const Matrix& matrix, double* pFi, double* pOmiga, double* pKapa);
	static double Mod(const Matrix& matrix); // Row mod
	static double Mod(const SimplexVertex& vertex);
};

#endif