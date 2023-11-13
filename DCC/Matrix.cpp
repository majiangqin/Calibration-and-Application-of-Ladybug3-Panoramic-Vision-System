// Implementation of matrix algorithm

#include "stdafx.h"
#include "Matrix.h"

#include "Warning.h"
#include "math.h"
#include "PicData.h"
#include "MathUtil.h"

#define ZERO   1.0e-10

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

SimplexVertex::SimplexVertex(int dim)
: m_nDim(dim)
{
	m_pVector = new double[dim];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::SimplexVertex");
	for (int ii = 0; ii < dim; ++ii)
	{
		m_pVector[ii] = 0.0;
	}
}

SimplexVertex::SimplexVertex(int dim, const double* pVector)
: m_nDim(dim)
{
	m_pVector = new double[dim];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::SimplexVertex");
	DBG_WARN_AND_RETURN_VOID_UNLESS(pVector != NULL, "Null vertex pointer in SimplexVertex::SimplexVertex");
	for (int ii = 0; ii < dim; ++ii)
	{
		m_pVector[ii] = pVector[ii];
	}
}

SimplexVertex::SimplexVertex(const SimplexVertex& vertex)
: m_nDim(vertex.m_nDim)
{
	m_pVector = new double[m_nDim];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::SimplexVertex");
	for (int ii = 0; ii < m_nDim; ++ii)
	{
		m_pVector[ii] = vertex.getEntry(ii);
	}
}

SimplexVertex::SimplexVertex(const Matrix& matrix)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(matrix.getCol() == 1, "Simplex vertex can only be assigned with one-col matrix");
	m_nDim = matrix.size();
	m_pVector = new double[m_nDim];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::SimplexVertex");
	for (int ii = 0; ii < m_nDim; ++ii)
	{
		m_pVector[ii] = matrix.getEntry(ii,0);
	}
}

SimplexVertex::SimplexVertex(const ImagePoint& point)
{
	m_nDim = 2;
	m_pVector = new double[2];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::SimplexVertex");
	m_pVector[0] = point.getXPos();
	m_pVector[1] = point.getYPos();
}

SimplexVertex::~SimplexVertex()
{
	if (m_pVector) delete[] m_pVector;
	m_nDim = 0;
}

void SimplexVertex::setDim(int dim)
{
	if (m_nDim != dim)
	{
		m_nDim = dim;
		if (m_pVector) delete[] m_pVector;
		m_pVector = new double[dim];
		DBG_WARN_AND_RETURN_VOID_UNLESS(m_pVector != NULL, "Memory allocation failure in SimplexVertex::setDim");
	}
}

double SimplexVertex::getEntry(int i) const
{
	DBG_WARN_AND_RETURN_UNLESS(i >= 0 && i < m_nDim, 0.0, "i exceeds the range of dimension in SimplexVertex::getEntry!");
	return m_pVector[i];
}

void SimplexVertex::getVector(double* pVector) const
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(pVector, "pVector is Null in SimplexVertex::getVector");
	for (int ii = 0; ii < m_nDim; ++ii)
	{
		pVector[ii] = m_pVector[ii];
	}
}

SimplexVertex SimplexVertex::getSubSpace(int start, int end) const
{
   if (start < 0 || start > end || end >= m_nDim)
   {
      DBG_WARN("start or end index exceed range in SimplexVertex::getSubSpace");
      return SimplexVertex();
   }
   SimplexVertex subspace(end-start+1);
   for (int ii = start, jj = 0; ii <= end; ++ii, ++jj)
   {
      subspace(jj) = getEntry(ii);
   }
   return subspace;
}

void SimplexVertex::setSubSpace(int start, int end, const SimplexVertex& vertex)
{
   if ( start < 0 || start > end || end >= m_nDim || (vertex.getDim() != end-start+1) )
   {
      DBG_WARN("start or end index exceed range in SimplexVertex::setSubSpace");
      return;
   }
   for (int ii = start, jj = 0; ii <= end; ++ii, ++jj)
   {
      setEntry(ii,vertex(jj));
   }
}

void SimplexVertex::setEntry(int i, double x)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(i >= 0 && i < m_nDim, "i exceeds the range of dimension in SimplexVertex::setEntry");
	m_pVector[i] = x;
}

SimplexVertex& SimplexVertex::operator =(const SimplexVertex& vertex)
{
	if( (m_nDim != vertex.m_nDim) && m_pVector ) delete[] m_pVector;
	if(vertex.m_pVector)
	{
		if (m_nDim != vertex.m_nDim)
		{
			m_nDim = vertex.m_nDim;
			m_pVector = new double[vertex.m_nDim];
		}
		if (!m_pVector)
		{
			DBG_WARN("Memory allocation failure in SimplexVertex::operator=");
		}
		else
		{
			for (int ii = 0; ii < m_nDim; ++ii)
			{
				m_pVector[ii] = vertex.getEntry(ii);
			}
		}

	}
	else
	{
		m_pVector = NULL;
		DBG_WARN("Null pointer of vertex in SimplexVertex::operator=");
	}
	return *this;
}

SimplexVertex& SimplexVertex::operator +=(const SimplexVertex& vertex)
{
	DBG_WARN_AND_RETURN_UNLESS(m_nDim == vertex.m_nDim, *this, "It is not a congruent simplex vertex");
	for (int ii = 0; ii < m_nDim; ++ii)
	{
		m_pVector[ii] += vertex.getEntry(ii);
	}
	return *this;
}

SimplexVertex& SimplexVertex::operator *=(double val)
{
	for (int ii = 0; ii < m_nDim; ++ii)
	{
		m_pVector[ii] *= val;
	}
	return *this;
}

double& SimplexVertex::operator ()(int i)
{
	if (i < 0 || i >= m_nDim)
		DBG_WARN("The entry can't be found in SimplexVertex::operator ()");
	return m_pVector[i];
}

double SimplexVertex::operator ()(int i) const
{
	if (i < 0 || i >= m_nDim)
		DBG_WARN("The entry can't be found in SimplexVertex::operator ()");
	return m_pVector[i];
}

SimplexVertex operator + (const SimplexVertex& vertex1, const SimplexVertex& vertex2)
{
	SimplexVertex vertex;
	if (!vertex1.m_pVector || !vertex2.m_pVector)
	{
		DBG_WARN("vertex1 or vertex2 has null data pointer in operator +");
	}
	else if (vertex1.m_nDim != vertex2.m_nDim)
	{
		DBG_WARN("vertex1 and vertex2 have different dimensions in operator +");
	}
	else
	{
		vertex.m_nDim = vertex1.m_nDim;
		vertex.m_pVector = new double[vertex1.m_nDim];
		if (!vertex.m_pVector)
		{
			DBG_WARN("Memory allocation failure in operator +");
		}
		else
		{
			for (int ii = 0; ii < vertex.m_nDim; ++ii)
			{
				vertex.m_pVector[ii] = vertex1.getEntry(ii) + vertex2.getEntry(ii);
			}
		}
	}
	return vertex;
}

SimplexVertex operator - (const SimplexVertex& vertex1, const SimplexVertex& vertex2)
{
	SimplexVertex vertex;
	if (!vertex1.m_pVector || !vertex2.m_pVector)
	{
		DBG_WARN("vertex1 or vertex2 has null data pointer in operator -");
	}
	else if (vertex1.m_nDim != vertex2.m_nDim)
	{
		DBG_WARN("vertex1 and vertex2 have different dimensions in operator -");
	}
	else
	{
		vertex.m_nDim = vertex1.m_nDim;
		vertex.m_pVector = new double[vertex1.m_nDim];
		if (!vertex.m_pVector)
		{
			DBG_WARN("Memory allocation failure in operator -");
		}
		else
		{
			for (int ii = 0; ii < vertex.m_nDim; ++ii)
			{
				vertex.m_pVector[ii] = vertex1.getEntry(ii) - vertex2.getEntry(ii);
			}
		}
	}
	return vertex;
}

SimplexVertex operator * (double val, const SimplexVertex& vertex)
{
	SimplexVertex vertex1;
	if (!vertex.m_pVector)
	{
		DBG_WARN("vertex has null data pointer in operator *");
	}
	else
	{
		vertex1.m_nDim = vertex.m_nDim;
		vertex1.m_pVector = new double[vertex.m_nDim];
		if (!vertex1.m_pVector)
		{
			DBG_WARN("Memory allocation failure in operator *");
		}
		else
		{
			for (int ii = 0; ii < vertex1.m_nDim; ++ii)
			{
				vertex1.m_pVector[ii] = val * vertex.getEntry(ii);
			}
		}
	}
	return vertex1;
}

Matrix::Matrix(int m, int n)
: m_row(m)
, m_col(n)
{
	m_data = new double[m*n];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_data != NULL, "Memory allocation failure in Matrix::Matrix");
	for (int ii = 0; ii < m*n; ++ii)
	{
		m_data[ii] = 0.0;
	}
}

Matrix::Matrix(int m, int n, const double *pData)
: m_row(m)
, m_col(n)
{
	m_data = new double[m*n];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_data != NULL, "Memory allocation failure in Matrix::Matrix");
	for (int ii = 0; ii < m*n; ++ii)
	{
		m_data[ii] = pData[ii];
	}
}

Matrix::Matrix(const Matrix& matrix)
: m_row(matrix.m_row)
, m_col(matrix.m_col)
{
	m_data = new double[m_row*m_col];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_data != NULL, "Memory allocation failure in Matrix::Matrix");
	for (int ii = 0; ii < m_row*m_col; ++ii)
	{
		m_data[ii] = matrix.m_data[ii];
	}
}

Matrix::Matrix(const SimplexVertex& vertex)
: m_row(vertex.getDim())
, m_col(1)
{
	m_data = new double[m_row];
	DBG_WARN_AND_RETURN_VOID_UNLESS(m_data != NULL, "Memory allocation failure in Matrix::Matrix");
	for (int ii = 0; ii < m_row; ++ii)
	{
		m_data[ii] = vertex.getEntry(ii);
	}
}

Matrix::~Matrix()
{
	if (m_data) delete[] m_data;
	m_row = 0;
	m_col = 0;
}

void Matrix::resize(int row, int col)
{
   if( (size() != row*col) && m_data ) delete[] m_data;
   m_row = row;
   m_col = col;
   m_data = new double[m_row*m_col];
   if (!m_data)
   {
      DBG_WARN("Memory allocation failure in Matrix::resize");
   }
   else
   {
      for (int ii = 0; ii < size(); ++ii)
      {
         m_data[ii] = 0.0;
      }
   }
}

double Matrix::getEntry(int m, int n) const
{
	DBG_WARN_AND_RETURN_UNLESS(m>=0&&m<m_row&&n>=0&&n<m_col, 0.0, "The data can't be found in Matrix::getEntry");
	return m_data[m*m_col+n];
}

void Matrix::setEntry(int m, int n, double val)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(m>=0&&m<m_row&&n>=0&&n<m_col, "The data can't be found in Matrix::setEntry");
	m_data[m*m_col+n] = val;
}

SimplexVertex Matrix::getRowVec(int row) const
{
	DBG_WARN_AND_RETURN_UNLESS(row>=0&&row<m_row, SimplexVertex(), "The row vector can't be found in Matrix::getRowVec");
	SimplexVertex rowVec(m_col);
	for (int jj = 0; jj < m_col; ++jj)
	{
		rowVec.setEntry(jj,getEntry(row,jj));
	}
	return rowVec;
}

void Matrix::setRowVec(int row, const SimplexVertex& vv)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(row>=0&&row<m_row, "The row vector can't be found in Matrix::setRowVec");
	for (int jj = 0; jj < m_col; ++jj)
	{
		setEntry(row,jj,vv.getEntry(jj));
	}
}

SimplexVertex Matrix::getColVec(int col) const
{
	DBG_WARN_AND_RETURN_UNLESS(col>=0&&col<m_col, SimplexVertex(), "The column vector can't be found in Matrix::getColVec");
	SimplexVertex colVec(m_row);
	for (int ii = 0; ii < m_row; ++ii)
	{
		colVec.setEntry(ii,getEntry(ii,col));
	}
	return colVec;
}

void Matrix::setColVec(int col, const SimplexVertex& vv)
{
	DBG_WARN_AND_RETURN_VOID_UNLESS(col>=0&&col<m_col, "The column vector can't be found in Matrix::setColVec");
	for (int ii = 0; ii < m_row; ++ii)
	{
		setEntry(ii,col,vv.getEntry(ii));
	}
}

Matrix& Matrix::operator =(const Matrix& matrix)
{
	if( (size() != matrix.size()) && m_data ) delete[] m_data;
	if(matrix.m_data)
	{
		m_row = matrix.m_row;
		m_col = matrix.m_col;
		m_data = new double[m_row*m_col];
		if (!m_data)
		{
			DBG_WARN("Memory allocation failure in Matrix::operator=");
		}
		else
		{
			for (int ii = 0; ii < size(); ++ii)
			{
				m_data[ii] = matrix.m_data[ii];
			}
		}
	}
	else
	{
		m_data = NULL;
		DBG_WARN("Null data pointer in Matrix::operator=");
	}
	return *this;
}

Matrix& Matrix::operator +=(const Matrix &matrix)
{
	DBG_WARN_AND_RETURN_UNLESS(m_row == matrix.m_row && m_col == matrix.m_col, *this, "It is not a congruent matrix");
	DBG_WARN_AND_RETURN_UNLESS(m_data, *this, "Null data pointer in Matrix::operator +=");
	for (int ii = 0; ii < size(); ++ii)
	{
		m_data[ii] += matrix.m_data[ii];
	}
	return *this;
}

Matrix& Matrix::operator *=(double val)
{
	DBG_WARN_AND_RETURN_UNLESS(m_data, *this, "Null data pointer in Matrix::operator *=");
	for (int ii = 0; ii < size(); ++ii)
	{
		m_data[ii] *= val;
	}
	return *this;
}

double& Matrix::operator ()(int i, int j)
{
	if (i < 0 || i >= m_row || j < 0 || j >= m_col)
		DBG_WARN("The entry can't be found in Matrix::operator ()");
	return m_data[i*m_col+j];
}

double Matrix::operator ()(int i, int j) const
{
	if (i < 0 || i >= m_row || j < 0 || j >= m_col)
		DBG_WARN("The entry can't be found in Matrix::operator ()");
	return m_data[i*m_col+j];
}

Matrix operator + (const Matrix& matrix1, const Matrix& matrix2)
{
	Matrix matrix;
	if (!matrix1.m_data || !matrix2.m_data)
	{
		DBG_WARN("One or both matrix has null data pointer in operator +");
	}
	else if (matrix1.m_row != matrix2.m_row || matrix1.m_col != matrix2.m_col)
	{
		DBG_WARN("They are not congruent matrix in operator +");
	}
	else
	{
		matrix.m_row = matrix1.m_row; matrix.m_col = matrix1.m_col;
		matrix.m_data = new double[matrix1.size()];
		if (!matrix.m_data)
		{
			DBG_WARN("Memory allocation failure in operator +");
		}
		else
		{
			for (int ii = 0; ii < matrix.size(); ++ii)
			{
				matrix.m_data[ii] = matrix1.m_data[ii] + matrix2.m_data[ii];
			}
		}
	}
	return matrix;
}

Matrix operator - (const Matrix& matrix1, const Matrix& matrix2)
{
	Matrix matrix;
	if (!matrix1.m_data || !matrix2.m_data)
	{
		DBG_WARN("One or both matrix has null data pointer in operator -");
	}
	else if (matrix1.m_row != matrix2.m_row || matrix1.m_col != matrix2.m_col)
	{
		DBG_WARN("They are not congruent matrix in operator -");
	}
	else
	{
		matrix.m_row = matrix1.m_row; matrix.m_col = matrix1.m_col;
		matrix.m_data = new double[matrix1.size()];
		if (!matrix.m_data)
		{
			DBG_WARN("Memory allocation failure in operator -");
		}
		else
		{
			for (int ii = 0; ii < matrix.size(); ++ii)
			{
				matrix.m_data[ii] = matrix1.m_data[ii] - matrix2.m_data[ii];
			}
		}
	}
	return matrix;
}

Matrix operator * (double val, const Matrix& matrix)
{
	Matrix mm;
	if (!matrix.m_data)
	{
		DBG_WARN("matrix has null data pointer in operator *");
	}
	else
	{
		mm.m_row = matrix.m_row; mm.m_col = matrix.m_col;
		mm.m_data = new double[matrix.size()];
		if (!mm.m_data)
		{
			DBG_WARN("Memory allocation failure in operator *");
		}
		else
		{
			for (int ii = 0; ii < mm.size(); ++ii)
			{
				mm.m_data[ii] = val * matrix.m_data[ii];
			}
		}
	}
	return mm;
}

Matrix operator * (const Matrix& matrix1, const Matrix& matrix2)
{
	Matrix matrix;
	if (!matrix1.m_data || !matrix2.m_data)
	{
		DBG_WARN("matrix has null data pointer in operator *");
	}
	else if (matrix1.m_col != matrix2.m_row)
	{
		DBG_WARN("Two matrix can't match");
	}
	else
	{
		matrix.m_row = matrix1.m_row; matrix.m_col = matrix2.m_col;
		matrix.m_data = new double[matrix.size()];
		if (!matrix.m_data)
		{
			DBG_WARN("Memory allocation failure in operator *");
		} 
		else
		{
			for (int ii = 0; ii < matrix.m_row; ++ii)
			{
				for (int jj = 0; jj < matrix.m_col; ++jj)
				{
					// Compute matrix[ii][jj]
					double val = 0.0;
					for (int kk = 0; kk < matrix1.m_col; ++kk)
					{
						val += matrix1(ii,kk) * matrix2(kk,jj);
					}
					matrix(ii,jj) = val;
				}
			}
		}
	}
	return matrix;
}

Matrix Transpose(const Matrix& matrix)
{
	Matrix mm(matrix.m_col,matrix.m_row);
	if (!matrix.m_data)
	{
		DBG_WARN("matrix has null data pointer in Transpose");
	}
	else
	{
		for (int ii = 0; ii < matrix.m_row; ++ii)
		{
			for (int jj = 0; jj < matrix.m_col; ++jj)
			{
				mm(jj,ii) = matrix(ii,jj);
			}
		}
	}
	return mm;
}

bool Matrix::LU(int* indx, bool* pSign)
{
	DBG_WARN_AND_RETURN_UNLESS(indx, false, "Null pointer is inputed in Matrix::LU");
	DBG_WARN_AND_RETURN_UNLESS(isSquare(), false, "It is not a square");

	const int nn = m_row;
	bool sign = true; // no row interchanges yet
	double* pRowScale = new double[nn]; // store the implicit scaling of each row

	for (int ii = 0; ii < nn; ++ii)
	{
		// Loop over rows to get the implicit scaling information
		double big = 0.0;
		for (int jj = 0; jj < nn; ++jj)
		{
			double temp = abs(getEntry(ii,jj));
			if (temp > big)
				big = temp;
		}
		if (big == 0.0)
		{
			if (pSign)	*pSign = sign;
			if (pRowScale) delete [] pRowScale;
			return false;
		}
		pRowScale[ii] = 1.0/big;
	}

	int imax;
	for (int jj = 0; jj < nn; ++jj)
	{
		// The loop over columns of Crout's method
		for (int ii = 0; ii < jj; ++ii)
		{
			// This is equation (2.3.12) except for ii = jj
			double sum = getEntry(ii,jj);
			for (int kk = 0; kk < ii; ++kk)
				sum -= getEntry(ii,kk)*getEntry(kk,jj);
			setEntry(ii,jj,sum);
		}
		double big = 0.0; // Initialize for the search for largest pivot element
		for (int ii = jj; ii < nn; ++ii)
		{
			// This is ii = jj of equation (2.3.12) and ii = jj+1...N of equation (2.3.13)
			double sum = getEntry(ii,jj);
			for (int kk = 0; kk < jj; ++kk)
				sum -= getEntry(ii,kk)*getEntry(kk,jj);
			setEntry(ii,jj,sum);
			double temp = pRowScale[ii]*abs(sum);
			if (temp > big)
			{
				big = temp;
				imax = ii;
			}
		}
		if (jj != imax)
		{
			// interchange rows
			for (int kk = 0; kk < nn; ++kk)
			{
				double temp = getEntry(imax,kk);
				setEntry(imax,kk,getEntry(jj,kk));
				setEntry(jj,kk,temp);
			}
			sign = !sign;
			pRowScale[imax] = pRowScale[jj];
		}
		indx[jj] = imax;

		// If the pivot element is zero the matrix is singular (at least to the precision of the algorithm).
		// For some applications on singular matrices, it is desirable to substitute TINY for zero.
		if (getEntry(jj,jj) == 0.0) setEntry(jj,jj,TINY);

		if (jj != nn-1)
		{
			double temp = 1.0 / getEntry(jj,jj);
			for (int ii = jj + 1; ii < nn; ++ii)
				setEntry(ii,jj,temp*getEntry(ii,jj)); // Divide by the pivot element
		}
	} // Go back for the next column in the reduction

	if (pSign)	*pSign = sign;
	if (pRowScale) delete [] pRowScale;
	return true;
}

namespace
{
	// Solves the set of n linear equations A*X = B. (2.3.6) and (2.3.7)
	void lubksb(const Matrix& mA, const int* indx, SimplexVertex* vB)
	{
		DBG_WARN_AND_RETURN_VOID_UNLESS(mA.isSquare(), "Matrix A is not a square in lubksb");
		DBG_WARN_AND_RETURN_VOID_UNLESS(vB != NULL && indx != NULL, "Null pointer input in lubksb");
		DBG_WARN_AND_RETURN_VOID_UNLESS(mA.getRow() == vB->getDim(), "mB has invalid row number in lubksb");

		const int nn = vB->getDim();
		int kk = -1;
		for (int ii = 0; ii < nn; ++ii)
		{
			// When kk is set to a positive value, it will become the index of the first nonvanishing element of B.
			// We now do the forward substitution, equation (2.3.6).
			int ip = indx[ii];
			double sum = vB->getEntry(ip);
			vB->setEntry(ip,vB->getEntry(ii));
			if (kk >= 0)
			{
				for (int jj = kk; jj < ii; ++jj)
					sum -= mA(ii,jj) * vB->getEntry(jj);
			}
			else if (sum)
			{
				// A nonzero element was encountered, so from now on we will have to do the sums in the loop above
				kk = ii;
			}
			vB->setEntry(ii,sum);
		}
		for (int ii = nn-1; ii >= 0; --ii)
		{
			double sum = vB->getEntry(ii);
			for (int jj = ii+1; jj < nn; ++jj)
				sum -= mA(ii,jj)*vB->getEntry(jj);
			vB->setEntry(ii,sum/mA(ii,ii));
		}
	}
}

double Det(const Matrix& matrix)
{
	DBG_WARN_AND_RETURN_UNLESS(matrix.isSquare(), 0.0, "It is not a square in Det");
	const int nn = matrix.m_row;
	int* indx = new int[nn];
	bool sign = true;
	double val = 1.0;
	Matrix mA(matrix);
	if (!mA.LU(indx,&sign))
	{
		if (indx) delete [] indx;
		return 0.0;
	}
	for (int jj = 0; jj < nn; ++jj)
	{
		val *= mA.getEntry(jj,jj);
	}
	val *= sign ? 1.0 : -1.0;
	if (indx) delete [] indx;
	return val;
}

Matrix Inv(const Matrix& matrix)
{
	DBG_WARN_AND_RETURN_UNLESS(matrix.isSquare(), Matrix(), "It is not a square in Inv");

	const int nn = matrix.m_row;
	Matrix mA(matrix); // original matrix
	Matrix iM(nn,nn); // invert matrix
	SimplexVertex vb(nn); // right-hand side vector
	int* indx = new int[nn];

	// Decompose matrix just once. Find inverse by columns.
	if (!mA.LU(indx))
	{
		DBG_WARN("It is a singular matrix");
		if (indx) delete [] indx;
		return iM;
	}
	for (int jj = 0; jj < nn; ++jj)
	{
		for (int ii = 0; ii < nn; ++ii)
		{
			vb.setEntry(ii, ii==jj ? 1.0 : 0.0);
		}
		lubksb(mA,indx,&vb);
		iM.setColVec(jj,vb);
	}
	if (indx) delete [] indx;
	return iM;
}

bool Matrix::Givens(Matrix& mL)
{
	DBG_WARN_AND_RETURN_UNLESS(m_row >= m_col, false, "The rank of this matrix is less than column number in Matrix::Givens");
	DBG_WARN_AND_RETURN_UNLESS(mL.m_row == m_row, false, "Invalid argument mL in Matrix::Givens");

	const int mm = m_row;
   const int nn = m_col;
   const int ss = min(m_row-1,m_col);
   
   for (int jj = 0; jj < ss; ++jj)
   {
      for (int ii = jj+1; ii < mm; ++ii)
      {
         double xi = getEntry(jj,jj);
         double xj = getEntry(ii,jj);
         if (xj == 0.0)
            continue;
         double domain = sqrt(xi*xi+xj*xj);
         if (domain == 0.0)
            return false;
         double ci = xi/domain, si = xj/domain;
         for (int kk = 0; kk < nn; ++kk)
         {
            double xk1 = getEntry(jj,kk);
            double xk2 = getEntry(ii,kk);
            if ( xk1 == 0.0 && xk2 == 0.0 )
               continue;
            setEntry(jj,kk,ci*xk1+si*xk2);
            setEntry(ii,kk,-si*xk1+ci*xk2);
         }
         for (int kk = 0; kk < mL.getCol(); ++kk)
         {
            double xk1 = mL(jj,kk);
            double xk2 = mL(ii,kk);
            if ( xk1 == 0.0 && xk2 == 0.0 )
               continue;
            mL(jj,kk) = ci*xk1+si*xk2;
            mL(ii,kk) = -si*xk1+ci*xk2;
         }
      }
   }
   return true;
}

namespace
{
	// Solve overdetermined equation
	void slvodeq(const Matrix& mA, SimplexVertex* vB)
	{
      const int nn = mA.getCol();
		for (int ii = nn-1; ii >= 0; --ii)
		{
			double sum = vB->getEntry(ii);
			for (int jj = ii+1; jj < nn; ++jj)
			{
				sum -= mA(ii,jj) * vB->getEntry(jj);
			}
			sum /= mA.getEntry(ii,ii);
			vB->setEntry(ii,sum);
		}
	}
}



//////////////////////////////////////////////////////////////////////////
// Implementation of LinearEquations
//////////////////////////////////////////////////////////////////////////
Matrix LinearEquations::solve(Matrix& mA, Matrix& mB)
{
	DBG_WARN_AND_RETURN_UNLESS(mA.getRow()==mB.getRow(), Matrix(), "They are not congruent matrix in LinearEquations::solve");
	
	const int mm = mA.getCol();
	const int nn = mB.getCol();
	if (mA.isSquare())
	{
		Matrix mX(mm,nn); // Matrix X
		int* indx = new int[mm];
		if (!mA.LU(indx)) // LU decomposition
		{
			// Go to SVD for singular matrix
			if (indx) delete [] indx;
			//Matrix smA(mA);
			//smA.SVD();
			return mX;
		}
		for (int jj = 0; jj < nn; ++jj)
		{
			SimplexVertex vb(mB.getColVec(jj));
			lubksb(mA,indx,&vb);
			mX.setColVec(jj,vb);
		}
		if (indx) delete [] indx;
		return mX;
	}
	else if (mA.getRow() > mA.getCol())
	{
		Matrix mX(mm,nn); // Matrix X
		if (!mA.Givens(mB))
		{
			DBG_WARN("Upper triangular matrix is singular in LinearEquations::solve");
			return mX;
		}
		for (int jj = 0; jj < nn; ++jj)
		{
			SimplexVertex vb(mB.getColVec(jj));
			slvodeq(mA,&vb);
         mX.setColVec(jj,vb);
		}
		return mX;
	}
	else
	{
		return Inv(Transpose(mA)*mA)*Transpose(mA)*mB;
	}
}


//////////////////////////////////////////////////////////////////////////
// Implementation of Matrix Utilities
//////////////////////////////////////////////////////////////////////////
Matrix MatrixUtil::generateRotateMatrix(double fi, double omiga, double kapa)
{
   fi = MathUtil::Radian(fi);
   omiga = MathUtil::Radian(omiga);
   kapa = MathUtil::Radian(kapa);
	double cfi = cos(fi), sfi = sin(fi);
	double comiga = cos(omiga), somiga = sin(omiga);
	double ckapa = cos(kapa), skapa = sin(kapa);
	double array[9] = { cfi*ckapa-sfi*somiga*skapa, -cfi*skapa-sfi*somiga*ckapa, -sfi*comiga,
							  comiga*skapa,					comiga*ckapa,					  -somiga,
							  sfi*ckapa+cfi*somiga*skapa, -sfi*skapa+cfi*somiga*ckapa, cfi*comiga };
	Matrix rotMatrix(3,3,array);
	return rotMatrix;
}

void MatrixUtil::computeRotateAngle(const Matrix& matrix, double* pFi, double* pOmiga, double* pKapa)
{
   DBG_WARN_AND_RETURN_VOID_UNLESS(pFi&&pOmiga&&pKapa, "Null pointer in MatrixUtil::computeRotateAngle");
   if (abs(matrix(2,2)) < ZERO)
   {
      if (-matrix(0,2)*matrix(2,2) >= 0)
         *pFi = PI/2.0;
      else
         *pFi = -PI/2.0;
   }
   else
   {
      *pFi = atan(-matrix(0,2)/matrix(2,2));
   }
   if (abs(matrix(1,2)) > 1.0)
   {
      if (-matrix(1,2) >= 0)
         *pOmiga = PI/2;
      else
         *pOmiga = -PI/2;
   } 
   else
   {
      *pOmiga = asin(-matrix(1,2));
   }
   if (abs(matrix(1,1)) < ZERO)
   {
      if (matrix(1,0)/matrix(1,1) >= 0)
         *pKapa = PI/2.0;
      else
         *pKapa = -PI/2.0;
   } 
   else
   {
      *pKapa = atan(matrix(1,0)/matrix(1,1));
   }
}

double MatrixUtil::Mod(const SimplexVertex& vertex)
{
	double sum = 0.0;
	for (int ii = 0; ii < vertex.getDim(); ++ii)
	{
		sum += vertex(ii)*vertex(ii);
	}
	return sqrt(sum);
}