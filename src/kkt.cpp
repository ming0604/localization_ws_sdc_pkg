#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream> 
#include <armadillo>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "kkt.h"

#define N 6

using namespace std;

//bool inverse(double A[N][N], double inverse[N][N]);

int kktInit(KKT_DATA *kktDataPtr) // set Fz0[4] and QAA0, then inverse QAA0
{
	arma::mat A(N,N,arma::fill::zeros);

	kktDataPtr->Fz0[0] = MASS*GRAVITY*LR/(LF+LR)/2;
	kktDataPtr->Fz0[1] = kktDataPtr->Fz0[0];
	kktDataPtr->Fz0[2] = MASS*GRAVITY*LF/(LF+LR)/2;
	kktDataPtr->Fz0[3] = kktDataPtr->Fz0[2];

	for(int i=0; i<4; i++){
		/*
		kktDataPtr->uFz0Square[i] = pow(kktDataPtr->u*kktDataPtr->Fz0[i],2.0);
		kktDataPtr->QAA0[i][i] = 2*kktDataPtr->C[i]/kktDataPtr->uFz0Square[i];
		kktDataPtr->QAA0[4][i] = 1.0;
		kktDataPtr->QAA0[i][4] = 1.0;
		kktDataPtr->QAA0[5][i] = (i%2==0) ? -(kktDataPtr->t_2) : (kktDataPtr->t_2);
		kktDataPtr->QAA0[i][5] = (i%2==0) ? -(kktDataPtr->t_2) : (kktDataPtr->t_2);
		*/
		kktDataPtr->uFz0Square[i] = pow(kktDataPtr->u*kktDataPtr->Fz0[i],2.0);
		A(i,i) = 2*kktDataPtr->C[i]/kktDataPtr->uFz0Square[i];
		A(4,i) = 1.0;
		A(i,4) = 1.0;
		A(5,i) = (i%2==0) ? -(kktDataPtr->t_2) : (kktDataPtr->t_2);
		A(i,5) = (i%2==0) ? -(kktDataPtr->t_2) : (kktDataPtr->t_2);
	}

	// std::cout << "kktInit" << std::endl;

	A.print("QAA0: ");

	if( arma::rank(A) == N ){
		arma::mat B = arma::inv(A);
		for(int i=0;i<N;i++){
			for(int j=0;j<N;j++){
				//kktDataPtr->QAA0[i][j] = A(i,j);
				kktDataPtr->QAA0inv[i][j] = B(i,j);
			}
		}
		B.print("QAA0inv: ");
		return 1;
	}
	else{
		cout << "QAA0 is singular" << endl;
		return 0;
	}
}

int kktOptimization(double FX, double M, KKT_DATA *kktDataPtr)
{
	double b[6] = {0.0};
	double xv[6] = {0.0}; //  QAA0*xv=b  =>   xv=QAA0inv*b
	int i,j;

	b[4] = FX;
	b[5] = M;

	// matrix multiply
	for(i=0;i<6;i++){
		for(j=0;j<6;j++){
			xv[i] += kktDataPtr->QAA0inv[i][j] * b[j];
		}
	}

	// check constraint
	for(i=0;i<4;i++){
		if(xv[i]*xv[i] > kktDataPtr->uFz0Square[i]){
			printf("[kkt: constraint met.]\n");
			return 0;
		}
	}

	for(i=0;i<4;i++)
		kktDataPtr->Fad[i] = xv[i];
	kktDataPtr->v[0] = xv[4];
	kktDataPtr->v[1] = xv[5];

	return 1;
}


#if false //no need compiling, using <armadillo> instead

//============================================================//
//                        matrix invert                       //
//============================================================//

// Function to get cofactor of A[p][q] in temp[][]. n is current 
// dimension of A[][] 
void getCofactor(double A[N][N], double temp[N][N], int p, int q, int n) 
{ 
	int i = 0, j = 0; 

	// Looping for each element of the matrix 
	for (int row = 0; row < n; row++) 
	{ 
		for (int col = 0; col < n; col++) 
		{ 
			// Copying into temporary matrix only those element 
			// which are not in given row and column 
			if (row != p && col != q) 
			{ 
				temp[i][j++] = A[row][col]; 

				// Row is filled, so increase row index and 
				// reset col index 
				if (j == n - 1) 
				{ 
					j = 0; 
					i++; 
				} 
			} 
		} 
	} 
} 

/* Recursive function for finding determinant of matrix. 
n is current dimension of A[][]. */
double determinant(double A[N][N], int n) 
{ 
	double D = 0; // Initialize result 

	// Base case : if matrix contains single element 
	if (n == 1) 
		return A[0][0]; 

	double temp[N][N]; // To store cofactors 

	double sign = 1; // To store sign multiplier 

	// Iterate for each element of first row 
	for (int f = 0; f < n; f++) 
	{ 
		// Getting Cofactor of A[0][f] 
		getCofactor(A, temp, 0, f, n); 
		D += sign * A[0][f] * determinant(temp, n - 1); 

		// terms are to be added with alternate sign 
		sign = -sign; 
	} 

	return D; 
} 

// Function to get adjoint of A[N][N] in adj[N][N]. 
void adjoint(double A[N][N],double adj[N][N]) 
{ 
	if (N == 1) 
	{ 
		adj[0][0] = 1; 
		return; 
	} 

	// temp is used to store cofactors of A[][] 
	int sign = 1;
	double temp[N][N]; 

	for (int i=0; i<N; i++) 
	{ 
		for (int j=0; j<N; j++) 
		{ 
			// Get cofactor of A[i][j] 
			getCofactor(A, temp, i, j, N); 

			// sign of adj[j][i] positive if sum of row 
			// and column indexes is even. 
			sign = ((i+j)%2==0)? 1: -1; 

			// Interchanging rows and columns to get the 
			// transpose of the cofactor matrix 
			adj[j][i] = (sign)*(determinant(temp, N-1)); 
		} 
	} 
} 

// Function to calculate and store inverse, returns false if 
// matrix is singular 
bool inverse(double A[N][N], double inverse[N][N]) 
{ 
	// Find determinant of A[][] 
	double det = determinant(A, N); 
	if (det == 0) 
	{ 
		cout << "Singular matrix, can't find its inverse"; 
		return false; 
	} 

	// Find adjoint 
	double adj[N][N]; 
	adjoint(A, adj); 

	// Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
	for (int i=0; i<N; i++) 
		for (int j=0; j<N; j++) 
			inverse[i][j] = adj[i][j]/float(det); 

	return true; 
}

#endif

