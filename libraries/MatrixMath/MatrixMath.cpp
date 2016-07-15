/*
 *  MatrixMath.cpp Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */

#include "MatrixMath.h"
#include <BigNumber.h>

#define NR_END 1

MatrixMath Matrix;			// Pre-instantiate

// Matrix Printing Routine
// Uses tabs to separate numbers under assumption printed float width won't cause problems
void MatrixMath::Print(BigNumber* A, int m, int n, String label){
    // A = input matrix (m x n)
    int i,j;
    Serial.println();
    Serial.println(label);
    for (i=0; i<m; i++){
        for (j=0;j<n;j++){
            //Serial.print(A[n*i+j],20);
            char * s = A[n*i+j].toString ();
            Serial.print (s); Serial.print(" ");
            free (s);
            //Serial.print("\t");
            //if (((n*i+j)==(n-1))||((n*i+j)==(2*n-1))||((n*i+j)==(3*n-1))){
            //    Serial.println("");
            //}
        }
        Serial.println();
    }
}

void MatrixMath::Copy(BigNumber* A, int n, int m, BigNumber* B)
{
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            B[n*i+j] = A[n*i+j];
        }
}

//Matrix Multiplication Routine
// C = A*B
void MatrixMath::Multiply(BigNumber* A, BigNumber* B, int m, int p, int n, BigNumber* C)
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            C[n*i+j]=0;
            for (k=0;k<p;k++)
                C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}


//Matrix Addition Routine
void MatrixMath::Add(BigNumber* A, BigNumber* B, int m, int n, BigNumber* C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}


//Matrix Subtraction Routine
void MatrixMath::Subtract(BigNumber* A, BigNumber* B, int m, int n, BigNumber* C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}


//Matrix Transpose Routine
void MatrixMath::Transpose(BigNumber* A, int m, int n, BigNumber* C)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[m*j+i]=A[n*i+j];
}

void MatrixMath::Scale(BigNumber* A, int m, int n, BigNumber k)
{
    for (int i=0; i<m; i++)
        for (int j=0; j<n; j++)
            A[n*i+j] = A[n*i+j]*k;
}


//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//	 NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int MatrixMath::Invert(BigNumber* A, int n)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow;		// keeps track of current pivot row
    int k,i,j;		// k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    BigNumber tmp;		// used for finding max value and making column swaps
    BigNumber zero=0;
    BigNumber one=1.0;
    
    for (k = 0; k < n; k++)
    {
        // find pivot row, the row with biggest entry in current column
        
        BigNumber tmp = 0;
        for (i = k; i < n; i++)
        {
            BigNumber two=2;
            if ((A[i*n+k].pow(two)).sqrt() >= tmp)	// 'Avoid using other functions inside abs(A[i*n+k])?'
            {
                tmp = (A[i*n+k].pow(two)).sqrt(); //abs of A[i*n+k]
                pivrow = i;
            }
        }
        
        // check for singular matrix
        if (A[pivrow*n+k] == zero)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }
        
        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < n; j++)
            {
                tmp = A[k*n+j];
                A[k*n+j] = A[pivrow*n+j];
                A[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;	// record row swap (even if no swap happened)
        
        tmp = one/A[k*n+k];	// invert pivot element
        A[k*n+k] = one;		// This element of input matrix becomes result matrix
        
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++)
        {
            A[k*n+j] = A[k*n+j]*tmp;
        }
        
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                tmp = A[i*n+k];
                A[i*n+k] = zero;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++)
                {
                    A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
                }
            }
        }
    }
    
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivrows[k]];
                A[i*n+pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}