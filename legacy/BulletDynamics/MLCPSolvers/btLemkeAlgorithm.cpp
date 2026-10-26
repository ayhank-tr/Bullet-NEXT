/* Copyright (C) 2004-2013 MBSim Development Team

Code was converted for the Bullet Continuous Collision Detection and Physics Library

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//The original version is here
//https://code.google.com/p/mbsim-env/source/browse/trunk/kernel/mbsim/numerics/linear_complementarity_problem/lemke_algorithm.cc
//This file is re-distributed under the ZLib license, with permission of the original author
//Math library was replaced from fmatvec to a the file src/LinearMath/btMatrixX.h
//STL/std::vector replaced by btAlignedObjectArray



#include "btLemkeAlgorithm.h"

#undef BT_DEBUG_OSTREAM
#ifdef BT_DEBUG_OSTREAM
using namespace std;
#endif //BT_DEBUG_OSTREAM

btlScalar btlMachEps()
{
	static bool calculated=false;
	static btlScalar machEps = btlScalar(1.);
	if (!calculated)
	{
		do {
			machEps /= btlScalar(2.0);
			// If next epsilon yields 1, then break, because current
			// epsilon is the machine epsilon.
		}
		while ((btlScalar)(1.0 + (machEps/btlScalar(2.0))) != btlScalar(1.0));
//		printf( "\nCalculated Machine epsilon: %G\n", machEps );
		calculated=true;
	}
	return machEps;
}

btlScalar btlEpsRoot() {
	
	static btlScalar epsroot = 0.;
	static bool alreadyCalculated = false;
	
	if (!alreadyCalculated) {
		epsroot = btlSqrt(btlMachEps());
		alreadyCalculated = true;
	}
	return epsroot;
}

	 

  btlVectorXu btlLemkeAlgorithm::solve(unsigned int maxloops /* = 0*/)
{
  
    
    steps = 0;

    int dim = m_q.size();
#ifdef BT_DEBUG_OSTREAM
    if(DEBUGLEVEL >= 1) {
      cout << "Dimension = " << dim << endl;
    }
#endif //BT_DEBUG_OSTREAM

	btlVectorXu solutionVector(2 * dim);
	solutionVector.setZero();
	  
	  //, INIT, 0.);

	btlMatrixXu ident(dim, dim);
	ident.setIdentity();
#ifdef BT_DEBUG_OSTREAM
	cout << m_M << std::endl;
#endif

	btlMatrixXu mNeg = m_M.negative();
	  
    btlMatrixXu A(dim, 2 * dim + 2);
	//
	A.setSubMatrix(0, 0, dim - 1, dim - 1,ident);
	A.setSubMatrix(0, dim, dim - 1, 2 * dim - 1,mNeg);
	A.setSubMatrix(0, 2 * dim, dim - 1, 2 * dim, -1.f);
	A.setSubMatrix(0, 2 * dim + 1, dim - 1, 2 * dim + 1,m_q);

#ifdef BT_DEBUG_OSTREAM
	cout << A << std::endl;
#endif //BT_DEBUG_OSTREAM


 //   btVectorXu q_;
 //   q_ >> A(0, 2 * dim + 1, dim - 1, 2 * dim + 1);

    btlAlignedObjectArray<int> basis;
    //At first, all w-values are in the basis
    for (int i = 0; i < dim; i++)
      basis.push_back(i);

	int pivotRowIndex = -1;
	btlScalar minValue = 1e30f;
	bool greaterZero = true;
	for (int i=0;i<dim;i++)
	{
		btlScalar v =A(i,2*dim+1);
		if (v<minValue)
		{
			minValue=v;
			pivotRowIndex = i;
		}
		if (v<0)
			greaterZero = false;
	}
	

	
  //  int pivotRowIndex = q_.minIndex();//minIndex(q_);     // first row is that with lowest q-value
    int z0Row = pivotRowIndex;           // remember the col of z0 for ending algorithm afterwards
    int pivotColIndex = 2 * dim;         // first col is that of z0

#ifdef BT_DEBUG_OSTREAM
    if (DEBUGLEVEL >= 3)
	{
    //  cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
      cout << "Basis: ";
      for (int i = 0; i < basis.size(); i++)
        cout << basis[i] << " ";
      cout << endl;
    }
#endif //BT_DEBUG_OSTREAM

	if (!greaterZero)
	{

      if (maxloops == 0) {
		  maxloops = 100;
//        maxloops = UINT_MAX; //TODO: not a really nice way, problem is: maxloops should be 2^dim (=1<<dim), but this could exceed UINT_MAX and thus the result would be 0 and therefore the lemke algorithm wouldn't start but probably would find a solution within less then UINT_MAX steps. Therefore this constant is used as a upper border right now...
      }

      /*start looping*/
      for(steps = 0; steps < maxloops; steps++) {

        GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);
#ifdef BT_DEBUG_OSTREAM
        if (DEBUGLEVEL >= 3) {
        //  cout << "A: " << A << endl;
          cout << "pivotRowIndex " << pivotRowIndex << endl;
          cout << "pivotColIndex " << pivotColIndex << endl;
          cout << "Basis: ";
          for (int i = 0; i < basis.size(); i++)
            cout << basis[i] << " ";
          cout << endl;
        }
#endif //BT_DEBUG_OSTREAM

        int pivotColIndexOld = pivotColIndex;

        /*find new column index */
        if (basis[pivotRowIndex] < dim) //if a w-value left the basis get in the correspondent z-value
          pivotColIndex = basis[pivotRowIndex] + dim;
        else
          //else do it the other way round and get in the corresponding w-value
          pivotColIndex = basis[pivotRowIndex] - dim;

        /*the column becomes part of the basis*/
        basis[pivotRowIndex] = pivotColIndexOld;

        pivotRowIndex = findLexicographicMinimum(A, pivotColIndex);

        if(z0Row == pivotRowIndex) { //if z0 leaves the basis the solution is found --> one last elimination step is necessary
          GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);
          basis[pivotRowIndex] = pivotColIndex; //update basis
          break;
      }

      }
#ifdef BT_DEBUG_OSTREAM
      if(DEBUGLEVEL >= 1) {
        cout << "Number of loops: " << steps << endl;
        cout << "Number of maximal loops: " << maxloops << endl;
      }
#endif //BT_DEBUG_OSTREAM

      if(!validBasis(basis)) {
        info = -1;
#ifdef BT_DEBUG_OSTREAM
        if(DEBUGLEVEL >= 1)
          cerr << "Lemke-Algorithm ended with Ray-Termination (no valid solution)." << endl;
#endif //BT_DEBUG_OSTREAM

        return solutionVector;
      }

    }
#ifdef BT_DEBUG_OSTREAM
    if (DEBUGLEVEL >= 2) {
     // cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
    }
#endif //BT_DEBUG_OSTREAM

    for (int i = 0; i < basis.size(); i++)
	{
      solutionVector[basis[i]] = A(i,2*dim+1);//q_[i];
	}

    info = 0;

    return solutionVector;
  }

  int btlLemkeAlgorithm::findLexicographicMinimum(const btlMatrixXu& A, const int & pivotColIndex) {
	  int RowIndex = 0;
	  int dim = A.rows();
	  btlAlignedObjectArray<btlVectorXu> Rows;
	  for (int row = 0; row < dim; row++) 
	  {

		  btlVectorXu vec(dim + 1);
		  vec.setZero();//, INIT, 0.)
		  Rows.push_back(vec);
		  btlScalar a = A(row, pivotColIndex);
		  if (a > 0) {
			  Rows[row][0] = A(row, 2 * dim + 1) / a;
			  Rows[row][1] = A(row, 2 * dim) / a;
			  for (int j = 2; j < dim + 1; j++)
				  Rows[row][j] = A(row, j - 1) / a;

#ifdef BT_DEBUG_OSTREAM
		//		if (DEBUGLEVEL) {
			//	  cout << "Rows(" << row << ") = " << Rows[row] << endl;
				// }
#endif 
		  }
	  }

	  for (int i = 0; i < Rows.size(); i++) 
	  {
		  if (Rows[i].nrm2() > 0.) {

			  int j = 0;
			  for (; j < Rows.size(); j++) 
			  {
				  if(i != j)
				  {
					  if(Rows[j].nrm2() > 0.)
					  {
						  btlVectorXu test(dim + 1);
						  for (int ii=0;ii<dim+1;ii++)
						  {
							  test[ii] = Rows[j][ii] - Rows[i][ii];
						  }

						  //=Rows[j] - Rows[i]
						  if (! LexicographicPositive(test))
							  break;
					  }
				  }
			  }

			  if (j == Rows.size()) 
			  {
				  RowIndex += i;
				  break;
			  }
		  }
	  }

	  return RowIndex;
  }

  bool btlLemkeAlgorithm::LexicographicPositive(const btlVectorXu & v)
{
    int i = 0;
  //  if (DEBUGLEVEL)
    //  cout << "v " << v << endl;

    while(i < v.size()-1 && fabs(v[i]) < btlMachEps())
      i++;
    if (v[i] > 0)
      return true;

    return false;
  }

void btlLemkeAlgorithm::GaussJordanEliminationStep(btlMatrixXu& A, int pivotRowIndex, int pivotColumnIndex, const btlAlignedObjectArray<int>& basis) 
{

	btlScalar a = -1 / A(pivotRowIndex, pivotColumnIndex);
#ifdef BT_DEBUG_OSTREAM
	cout << A << std::endl;
#endif

    for (int i = 0; i < A.rows(); i++)
	{
      if (i != pivotRowIndex)
	  {
        for (int j = 0; j < A.cols(); j++)
		{
          if (j != pivotColumnIndex)
		  {
			  btlScalar v = A(i, j);
			  v += A(pivotRowIndex, j) * A(i, pivotColumnIndex) * a;
            A.setElem(i, j, v);
		  }
		}
	  }
	}

#ifdef BT_DEBUG_OSTREAM
	cout << A << std::endl;
#endif //BT_DEBUG_OSTREAM
    for (int i = 0; i < A.cols(); i++) 
	{
      A.mulElem(pivotRowIndex, i,-a);
    }
#ifdef BT_DEBUG_OSTREAM
	cout << A << std::endl;
#endif //#ifdef BT_DEBUG_OSTREAM

    for (int i = 0; i < A.rows(); i++)
	{
      if (i != pivotRowIndex)
	  {
        A.setElem(i, pivotColumnIndex,0);
	  }
	}
#ifdef BT_DEBUG_OSTREAM
	cout << A << std::endl;
#endif //#ifdef BT_DEBUG_OSTREAM
  }

  bool btlLemkeAlgorithm::greaterZero(const btlVectorXu & vector)
{
    bool isGreater = true;
    for (int i = 0; i < vector.size(); i++) {
      if (vector[i] < 0) {
        isGreater = false;
        break;
      }
    }

    return isGreater;
  }

  bool btlLemkeAlgorithm::validBasis(const btlAlignedObjectArray<int>& basis) 
  {
    bool isValid = true;
    for (int i = 0; i < basis.size(); i++) {
      if (basis[i] >= basis.size() * 2) { //then z0 is in the base
        isValid = false;
        break;
      }
    }

    return isValid;
  }


