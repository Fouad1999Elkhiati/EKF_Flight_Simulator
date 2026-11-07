/**
 * @author  Fouad El khiati
 * @website http://stm32f4-discovery.net
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   Hardware-agnostic attitude Extended Kalman Filter (multiplicative formulation) for STM32Fxxx
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
#ifndef EKF_H
#define EKF_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup EKF
 * @brief    Attitude Extended Kalman Filter estimating roll, pitch, yaw and gyro bias
 * @{
 *
 * This library estimates the attitude of a 9-axis IMU (gyroscope, accelerometer, magnetometer).
 * It has no platform dependencies: it compiles unchanged on a Cortex-M (STM32F0xx, STM32F4xx, STM32F7xx),
 * on a PC for offline testing, or anywhere else with a C99 compiler and math.h.
 *
 * \par How it works
 *
 * The filter has six error states: 3 attitude errors and 3 gyro biases.
 * The quaternion is carried outside the state vector and corrected multiplicatively,
 * so its norm constraint is never violated.
 *
 * Call \ref ekf_predict at every gyro sample, then \ref ekf_update_accel and/or
 * \ref ekf_update_mag whenever those measurements are available.
 * Read the result with \ref ekf_euler.
 *
 * \par Frame convention
 *
 * q rotates body -> navigation. The navigation frame is ENU-like, with gravity read as +1 g
 * on Z when the board is level, matching what an MPU9250 reports lying flat.
 *
 * \par Example
 *
\verbatim
    ekf_t f;
    ekf_init(&f, &cfg);
    while (1) {
        ekf_predict(&f, gyro, dt);
        ekf_update_accel(&f, acc);
        ekf_update_mag(&f, mag);
        ekf_euler(&f, &roll, &pitch, &yaw);
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
 - math.h
 - string.h
\endverbatim
 */

/**
 * @defgroup EKF_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Filter tuning and reference configuration
 */
typedef struct {
    float gyro_noise;   /*!< Gyro white noise, in units of rad/s/sqrt(Hz) */
    float bias_noise;   /*!< Gyro bias random walk, in units of rad/s^2/sqrt(Hz) */
    float accel_noise;  /*!< Accelerometer measurement noise, normalised units */
    float mag_noise;    /*!< Magnetometer measurement noise, normalised units */
    float mag_ref[3];   /*!< Unit magnetic reference vector, navigation frame */
} ekf_cfg_t;

/**
 * @brief  Main EKF working structure
 */
typedef struct {
    float q[4];         /*!< Attitude quaternion w x y z, body -> nav, unit norm */
    float bias[3];      /*!< Estimated gyro bias, in units of rad/s */
    float P[36];        /*!< 6x6 error covariance, row-major */
    ekf_cfg_t cfg;      /*!< Copy of the configuration passed to \ref ekf_init */
} ekf_t;

/**
 * @}
 */

/**
 * @defgroup EKF_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes the filter with a level attitude, zero bias and a diagonal initial covariance.
 * @param  *f: Pointer to \ref ekf_t empty structure
 * @param  *cfg: Pointer to \ref ekf_cfg_t configuration. It is copied, so it may be a temporary.
 * @retval None
 */
void ekf_init(ekf_t *f, const ekf_cfg_t *cfg);

/**
 * @brief  Propagates the attitude and covariance with a gyro sample.
 * @param  *f: Pointer to \ref ekf_t structure
 * @param  gyro: Gyro data for X, Y and Z axis. These parameters must be in units of rad/s.
 * @param  dt: Time since the previous call, in units of seconds
 * @retval None
 */
void ekf_predict(ekf_t *f, const float gyro[3], float dt);

/**
 * @brief  Corrects roll and pitch from the gravity vector.
 * @note   The update is skipped when the sample magnitude is not close to 1 g,
 *         which rejects updates during linear acceleration.
 * @param  *f: Pointer to \ref ekf_t structure
 * @param  acc: Accelerometer data for X, Y and Z axis, in units of gees. It is normalised internally.
 * @retval Update status:
 *            - 1: Update applied
 *            - 0: Sample rejected, nothing changed
 */
int ekf_update_accel(ekf_t *f, const float acc[3]);

/**
 * @brief  Corrects heading from the magnetic field vector.
 * @note   mag must already be hard/soft-iron calibrated, see \ref MAGCAL.
 * @param  *f: Pointer to \ref ekf_t structure
 * @param  mag: Calibrated magnetic data for X, Y and Z axis
 * @retval Update status:
 *            - 1: Update applied
 *            - 0: Degenerate sample, nothing changed
 */
int ekf_update_mag(ekf_t *f, const float mag[3]);

/**
 * @brief  Extracts Euler angles from the current attitude.
 * @param  *f: Pointer to \ref ekf_t structure
 * @param  *roll: Pointer to store roll angle in units of radians. May be NULL.
 * @param  *pitch: Pointer to store pitch angle in units of radians. May be NULL.
 * @param  *yaw: Pointer to store yaw angle in units of radians. May be NULL.
 * @retval None
 */
void ekf_euler(const ekf_t *f, float *roll, float *pitch, float *yaw);

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

#endif /* EKF_H */
