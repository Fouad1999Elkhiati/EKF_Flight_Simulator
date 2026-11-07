/**
 * @author  Fouad El khiati
 * @website http://stm32f4-discovery.net
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   Gaussian elimination library STM32Fxxx
 *
@verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen MAJERLE

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef GAUSS_ELIMINATION_H_
#define GAUSS_ELIMINATION_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup Gauss_elimination
 * @brief    Matrix inversion and linear system solving by Gaussian elimination
 * @{
 *
 * This library allows you to operate Matrix inversion and linear system solving.
 * It works on STM32F0xx, STM32F4xx and STM32F7xx devices
 *
 * \par How it works
 *
 * It implements the standard Gauss method with partial pivoting on systems of any size n.
 * All matrices are n x n, stored row-major in a flat float array.
 *
 * \ref determinant reduces the matrix to upper-triangular form, then \ref inversion
 * completes the back substitution. They must be called in that order.
 *
 * \par Example
 *
\verbatim
    float A[n*n];    //Matrix to invert, destroyed
    float B[n*n];    //Must be the identity matrix on input, holds A^-1 on output
    float x[n];      //Right-hand side on input, solution of Ax = b on output

    if (determinant(A, B, x, n) != 0.0f) {
        inversion(A, B, x, n);
    }
\endverbatim
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - stdio.h
 - math.h
 - stdlib.h
 - time.h
\endverbatim
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/**
 * @defgroup GAUSS_elimination_macros
 * @brief    Library defines
 * @{
 */

/* GAUSS_EPS is defined in Gauss_elimination.c: magnitude below which a pivot is treated as zero (1e-6) */

/**
 * @}
 */

/**
 * @defgroup GAUSS_elimination_functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Swaps the values of two floats through pointers.
 * @param  *a: first float value
 * @param  *b: second float value
 * @retval None
 */
void swap(float *a, float *b);

/**
 * @brief  Swaps rows i and j of an n-column matrix stored row-major.
 * @param  *arr: Pointer to matrix
 * @param  i: index of row i
 * @param  j: index of row j
 * @param  n: number of elements to swap between rows
 * @retval None
 */
void ligne_swap(float* arr, int i, int j, int n);

/**
 * @brief  Partial pivoting: searches rows [pivot, n) for the largest-magnitude entry
 *         in column pivot and swaps that row into the pivot position.
 * @note   Searching only downward leaves already-eliminated rows untouched; choosing the largest
 *         magnitude keeps the elimination ratios small and limits rounding error.
 * @param  pivot: index of the current pivot row and column
 * @param  *a: Pointer to matrix being reduced
 * @param  *b: Pointer to augmented matrix, receives the same row swaps
 * @param  *tb: Pointer to right-hand side vector, receives the same row swaps
 * @param  n: matrix size
 * @param  *swap_cnt: Pointer to swap counter, incremented when a swap occurs,
 *                    so the determinant sign can be corrected afterwards
 * @retval None
 */
void partial_pivot(int pivot, float* a, float* b, float* tb, int n, int* swap_cnt);

/**
 * @brief  Zeroes the column below the pivot, applying the same row operations
 *         to the augmented matrix b and the right-hand side tb.
 * @param  pivot: index of the current pivot row and column
 * @param  *a: Pointer to matrix being reduced
 * @param  *b: Pointer to augmented matrix
 * @param  *tb: Pointer to right-hand side vector
 * @param  n: matrix size
 * @retval None
 */
void matrix_elimination(int pivot, float* a, float* b, float* tb, int n);

/**
 * @brief  Reduces a to upper-triangular form and returns its determinant.
 * @note   Destroys a, and applies the same operations to b and tb.
 *         Must be called before \ref inversion.
 * @param  *a: Pointer to matrix, destroyed
 * @param  *b: Pointer to augmented matrix, must hold the identity matrix to obtain the inverse
 * @param  *tb: Pointer to right-hand side vector
 * @param  n: matrix size
 * @retval Determinant of a, or 0 if the matrix is singular
 */
float determinant(float* a, float* b, float* tb, int n);

/**
 * @brief  Back substitution.
 * @note   Only valid if \ref determinant returned a non-zero value.
 * @param  *a: Pointer to upper-triangular matrix produced by \ref determinant
 * @param  *b: Pointer to augmented matrix, holds the inverse of the original matrix on return
 * @param  *tb: Pointer to right-hand side vector, holds the solution of Ax = b on return
 * @param  n: matrix size
 * @retval None
 */
void inversion(float* a, float* b, float* tb, int n);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* GAUSS_ELIMINATION_H_ */
