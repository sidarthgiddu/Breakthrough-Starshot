/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */

#ifndef MatrixMath_h
#define MatrixMath_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <BigNumber.h>

class MatrixMath
{
public:
    //MatrixMath();
    void Print(BigNumber* A, int m, int n, String label);
    void Copy(BigNumber* A, int n, int m, BigNumber* B);
    void Multiply(BigNumber* A, BigNumber* B, int m, int p, int n, BigNumber* C);
    void Add(BigNumber* A, BigNumber* B, int m, int n, BigNumber* C);
    void Subtract(BigNumber* A, BigNumber* B, int m, int n, BigNumber* C);
    void Transpose(BigNumber* A, int m, int n, BigNumber* C);
    void Scale(BigNumber* A, int m, int n, BigNumber k);
    int Invert(BigNumber* A, int n);
};

extern MatrixMath Matrix;
#endif