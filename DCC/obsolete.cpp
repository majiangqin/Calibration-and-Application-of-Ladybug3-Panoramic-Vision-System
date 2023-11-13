#include "stdafx.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//typedef SimplexVertex (*Func)(const SimplexVertex&);

//class Matrix
//{
//public:
//	friend double Det_old(const Matrix& matrix); // obsolete determinant algorithm
//
//private:
//	int m_row;
//	int m_col;
//	double* m_data;
//};

//double Det_old(const Matrix& matrix)
//{
//	DBG_WARN_AND_RETURN_UNLESS(matrix.m_row == matrix.m_col, 0.0, "It is not a square");
//	Matrix mm(matrix);
//	double detVal = 1.0;
//	double max = 0.0;
//	const int nn = mm.m_row;	
//	
//	for (int ii = 0, ik = 0; ii < nn; ++ii) // LU
//	{
//		ik = ii; // row index of column main element
//		max = abs(mm.getEntry(ii,ii));
//		for (int jj = ii+1; jj < nn; ++jj)
//		{
//			if (max < abs(mm.getEntry(jj,ii)))
//			{
//				max = abs(mm.getEntry(jj,ii));
//				ik = jj;
//			}
//		}
//		if (max < TINY)
//		{
//			return 0.0;
//		}
//		else if (ik != ii)
//		{
//			// substitute row ik for row ii
//			for (int kk = 0; kk < nn; ++kk)
//			{
//				double valTmp = mm.getEntry(ii,kk);
//				mm.setEntry(ii,kk,mm.getEntry(ik,kk));
//				mm.setEntry(ik,kk,valTmp);
//			}
//			detVal *= -1.0;
//		}
//		for (int kk = ii+1; kk < nn; ++kk)
//		{
//			double factor = mm.getEntry(kk,ii) / mm.getEntry(ii,ii);
//			mm.setEntry(kk,ii,0.0);
//			for (int jj = ii+1; jj < nn; ++jj)
//			{
//				double valTmp = mm.getEntry(kk,jj) - factor*mm.getEntry(ii,jj);
//				mm.setEntry(kk,jj,valTmp);
//			}
//		}
//		detVal *= mm.getEntry(ii,ii);
//	}
//
//	return detVal;
//}

//bool Matrix::QR(Matrix* pQT)
//{
//   // Initiate the transpose of Q as E
//   for (int ii = 0; ii < pQT->getRow(); ++ii)
//   {
//      for (int jj = 0; jj < pQT->getCol(); ++jj)
//      {
//         if (ii == jj)	pQT->setEntry(ii,jj,1.0); 
//         else	pQT->setEntry(ii,jj,0.0);
//      }
//   }
//
//   const double ss = min(m_row-1,m_col);
//   for (int ii = 0; ii < ss; ++ii)
//   {
//      // Eliminate elements below diagonal element in every column
//      for (int jj = ii+1; jj < m_row; ++jj)
//      {
//         double xi = getEntry(ii,ii);
//         double xj = getEntry(jj,ii);
//         if ( xj != 0.0 )
//         {
//            double temp = sqrt(xi*xi+xj*xj);
//            double ck = xi/temp, sk = xj/temp;
//            setEntry(jj,ii,0.0); setEntry(ii,ii,ck*xi+sk*xj);
//            if ( getEntry(ii,ii) == 0.0 )
//            {
//               return false;
//            }
//            for (int kk = ii+1; kk < m_col; ++kk)
//            {
//               double xik = getEntry(ii,kk);
//               double xjk = getEntry(jj,kk);
//               if ( xik == 0.0 && xjk == 0.0 )
//                  continue;
//               setEntry(ii,kk,ck*xik+sk*xjk);
//               setEntry(jj,kk,ck*xjk-sk*xik);
//            }
//            for (int kk = 0; kk < pQT->getCol(); ++kk)
//            {
//               double yik = pQT->getEntry(ii,kk);
//               double yjk = pQT->getEntry(jj,kk);
//               if ( yik == 0.0 && yjk == 0.0 )
//                  continue;
//               pQT->setEntry(ii,kk,ck*yik+sk*yjk);
//               pQT->setEntry(jj,kk,ck*yjk-sk*yik);
//            }
//         }
//         else
//         {
//            continue;
//         }
//      }
//   }
//   return true;
//}
