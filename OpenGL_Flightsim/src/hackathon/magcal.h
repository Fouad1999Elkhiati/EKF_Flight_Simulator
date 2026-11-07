/**
 * @author  Fouad El khiati
 * @website http://stm32f4-discovery.net
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   Magnetometer hard/soft-iron calibration library by ellipsoid fitting for STM32Fxxx
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
#ifndef MAGCAL_H
#define MAGCAL_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup MAGCAL
 * @brief    Magnetometer hard/soft-iron calibration by least-squares ellipsoid fitting
 * @{
 *
 * This library calibrates a 3-axis magnetometer (for example the AK8963 inside the MPU9250).
 * It works on STM32F0xx, STM32F4xx and STM32F7xx devices, and on any platform with a C compiler.
 *
 * \par How it works
 *
 * Raw magnetometer readings trace an off-centre, stretched ellipsoid rather than a sphere:
 * hard-iron effects (nearby ferrous material) shift the centre, soft-iron effects scale the axes.
 * Uncalibrated, the heading derived from them is unusable.
 *
 * This library fits the axis-aligned ellipsoid
 *
\verbatim
    x^2 + B y^2 + C z^2 + P x + Q y + R z + D = 0
\endverbatim
 *
 * by least squares. Each sample contributes one row to the design matrix; the normal
 * equations M'M v = M'w form a 6x6 dense system, solved once with Gaussian elimination.
 *
 * The accumulators are fixed size, so memory does not grow with the number of samples:
 * samples are folded in and discarded.
 *
 * \par Example
 *
\verbatim
    magcal_t c;
    magcal_reset(&c);
    while (collecting) magcal_add(&c, mx, my, mz);   //Rotate the sensor in all axes
    if (magcal_solve(&c)) magcal_apply(&c, raw, out);
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
 - Gauss_elimination
 - math.h
 - string.h
\endverbatim
 */

/**
 * @defgroup MAGCAL_Macros
 * @brief    Library defines
 * @{
 */

#define MAGCAL_MIN_SAMPLES  100    /*!< Minimal number of samples before \ref magcal_solve accepts to fit */

/**
 * @}
 */

/**
 * @defgroup MAGCAL_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Main magnetometer calibration working structure
 */
typedef struct {
    double   ata[36];      /*!< 6x6 normal matrix M'M, accumulated, row-major */
    double   atb[6];       /*!< 6-vector right-hand side M'w, accumulated */
    unsigned count;        /*!< Number of samples folded in so far */

    /* Results, valid after magcal_solve returns 1 */
    float offset[3];       /*!< Hard-iron centre, in the same units as the raw samples */
    float scale[3];        /*!< Per-axis soft-iron normalisation factor */
    int   valid;           /*!< Set to 1 when offset and scale hold a solved calibration */
} magcal_t;

/**
 * @}
 */

/**
 * @defgroup MAGCAL_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Clears all accumulators. Call before collecting samples.
 * @note   Offset is reset to 0 and scale to 1, so \ref magcal_apply is a pass-through until solved.
 * @param  *c: Pointer to \ref magcal_t structure
 * @retval None
 */
void magcal_reset(magcal_t *c);

/**
 * @brief  Folds one raw sample into the normal equations.
 * @note   Rotate the sensor through as many orientations as possible.
 *         A fit from samples clustered in one attitude is ill-conditioned.
 * @param  *c: Pointer to \ref magcal_t structure
 * @param  mx, my, mz: Raw magnetic data for X, Y and Z axis respectively
 * @retval None
 */
void magcal_add(magcal_t *c, float mx, float my, float mz);

/**
 * @brief  Solves the 6x6 system and derives offset and scale.
 * @param  *c: Pointer to \ref magcal_t structure
 * @retval Calibration status:
 *            - 1: Success, calibration is valid
 *            - 0: Too few samples (less than \ref MAGCAL_MIN_SAMPLES) or the fit is degenerate
 */
int magcal_solve(magcal_t *c);

/**
 * @brief  Applies a solved calibration to a raw sample.
 * @note   Output is normalised to roughly unit length.
 * @param  *c: Pointer to \ref magcal_t structure
 * @param  raw: Raw magnetic sample, X, Y and Z axis
 * @param  out: Calibrated output, X, Y and Z axis. It may alias raw.
 * @retval None
 */
void magcal_apply(const magcal_t *c, const float raw[3], float out[3]);

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

#endif /* MAGCAL_H */
