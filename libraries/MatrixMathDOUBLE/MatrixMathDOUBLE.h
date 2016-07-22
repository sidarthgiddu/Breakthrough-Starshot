/*
 *  MatrixMathDOUBLE.h Library for Matrix Math using doubles
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */

#ifndef MatrixMathDOUBLE_h
#define MatrixMathDOUBLE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MatrixMathDOUBLE
{
public:
    //MatrixMathDOUBLE();
    void Print(double* A, int m, int n, String label);
    void Copy(double* A, int n, int m, double* B);
    void Multiply(double* A, double* B, int m, int p, int n, double* C);
    void Add(double* A, double* B, int m, int n, double* C);
    void Subtract(double* A, double* B, int m, int n, double* C);
    void Transpose(double* A, int m, int n, double* C);
    void Scale(double* A, int m, int n, double k);
    int Invert(double* A, int n);
};

extern MatrixMathDOUBLE Matrix;
#endif