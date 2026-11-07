/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "ekf.h"
#include <math.h>
#include <string.h>

#define N 6                       /* error state dimension */
#define P_(i,j) f->P[(i)*N + (j)]

/* ---------------------------------------------------------------- */
/* Small helpers                                                      */
/* ---------------------------------------------------------------- */

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-9f) {
        n = 1.0f / n;
        q[0] *= n; q[1] *= n; q[2] *= n; q[3] *= n;
    } else {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    }
}

/* r = a (x) b, Hamilton product, w-first */
static void quat_mul(const float a[4], const float b[4], float r[4])
{
    r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* R maps body -> nav, row-major 3x3 */
static void quat_to_R(const float q[4], float R[9])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    R[0] = 1.0f - 2.0f*(y*y + z*z);
    R[1] =        2.0f*(x*y - w*z);
    R[2] =        2.0f*(x*z + w*y);

    R[3] =        2.0f*(x*y + w*z);
    R[4] = 1.0f - 2.0f*(x*x + z*z);
    R[5] =        2.0f*(y*z - w*x);

    R[6] =        2.0f*(x*z - w*y);
    R[7] =        2.0f*(y*z + w*x);
    R[8] = 1.0f - 2.0f*(x*x + y*y);
}

/* out = R^T * v  -- rotates a nav-frame vector into the body frame */
static void R_transpose_mul(const float R[9], const float v[3], float out[3])
{
    out[0] = R[0]*v[0] + R[3]*v[1] + R[6]*v[2];
    out[1] = R[1]*v[0] + R[4]*v[1] + R[7]*v[2];
    out[2] = R[2]*v[0] + R[5]*v[1] + R[8]*v[2];
}

/* Closed-form 3x3 inverse. ~15 flops, no branches in the hot path --
   substantially cheaper than general Gaussian elimination at this size.
   Returns 0 if the matrix is singular. */
static int inv3x3(const float m[9], float out[9])
{
    float c0 = m[4]*m[8] - m[5]*m[7];
    float c1 = m[5]*m[6] - m[3]*m[8];
    float c2 = m[3]*m[7] - m[4]*m[6];
    float det = m[0]*c0 + m[1]*c1 + m[2]*c2;
    float id;

    if (fabsf(det) < 1e-12f) return 0;
    id = 1.0f / det;

    out[0] = c0 * id;
    out[1] = (m[2]*m[7] - m[1]*m[8]) * id;
    out[2] = (m[1]*m[5] - m[2]*m[4]) * id;
    out[3] = c1 * id;
    out[4] = (m[0]*m[8] - m[2]*m[6]) * id;
    out[5] = (m[2]*m[3] - m[0]*m[5]) * id;
    out[6] = c2 * id;
    out[7] = (m[1]*m[6] - m[0]*m[7]) * id;
    out[8] = (m[0]*m[4] - m[1]*m[3]) * id;
    return 1;
}

static int normalize3(const float in[3], float out[3])
{
    float n = sqrtf(in[0]*in[0] + in[1]*in[1] + in[2]*in[2]);
    if (n < 1e-6f) return 0;
    n = 1.0f / n;
    out[0] = in[0]*n; out[1] = in[1]*n; out[2] = in[2]*n;
    return 1;
}

/* ---------------------------------------------------------------- */
/* Public API                                                         */
/* ---------------------------------------------------------------- */

void ekf_init(ekf_t *f, const ekf_cfg_t *cfg)
{
    int i;

    f->cfg = *cfg;

    f->q[0] = 1.0f; f->q[1] = f->q[2] = f->q[3] = 0.0f;
    f->bias[0] = f->bias[1] = f->bias[2] = 0.0f;

    memset(f->P, 0, sizeof(f->P));
    for (i = 0; i < 3; i++) {
        P_(i, i)         = 0.1f;    /* attitude uncertainty, rad^2 */
        P_(i+3, i+3)     = 0.01f;   /* bias uncertainty, (rad/s)^2 */
    }
}

void ekf_predict(ekf_t *f, const float gyro[3], float dt)
{
    float w[3], dq[4], qn[4];
    float Phi[N*N], tmp[N*N], Pnew[N*N];
    int i, j, k;

    /* Bias-corrected angular rate */
    w[0] = gyro[0] - f->bias[0];
    w[1] = gyro[1] - f->bias[1];
    w[2] = gyro[2] - f->bias[2];

    /* Quaternion integration, small-angle increment */
    dq[0] = 1.0f;
    dq[1] = 0.5f * w[0] * dt;
    dq[2] = 0.5f * w[1] * dt;
    dq[3] = 0.5f * w[2] * dt;
    quat_mul(f->q, dq, qn);
    memcpy(f->q, qn, sizeof(qn));
    quat_normalize(f->q);

    /* State transition for the error state:
         d(dtheta)/dt = -[w x] dtheta - dbias
         d(dbias)/dt  = 0
       Phi = I + F dt  */
    memset(Phi, 0, sizeof(Phi));
    for (i = 0; i < N; i++) Phi[i*N + i] = 1.0f;

    Phi[0*N + 1] =  w[2] * dt;   Phi[0*N + 2] = -w[1] * dt;
    Phi[1*N + 0] = -w[2] * dt;   Phi[1*N + 2] =  w[0] * dt;
    Phi[2*N + 0] =  w[1] * dt;   Phi[2*N + 1] = -w[0] * dt;

    Phi[0*N + 3] = -dt;
    Phi[1*N + 4] = -dt;
    Phi[2*N + 5] = -dt;

    /* tmp = Phi * P */
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) s += Phi[i*N + k] * f->P[k*N + j];
            tmp[i*N + j] = s;
        }

    /* Pnew = tmp * Phi^T */
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) s += tmp[i*N + k] * Phi[j*N + k];
            Pnew[i*N + j] = s;
        }

    /* Add process noise */
    {
        float qg = f->cfg.gyro_noise * f->cfg.gyro_noise * dt;
        float qb = f->cfg.bias_noise * f->cfg.bias_noise * dt;
        for (i = 0; i < 3; i++) {
            Pnew[i*N + i]         += qg;
            Pnew[(i+3)*N + (i+3)] += qb;
        }
    }

    memcpy(f->P, Pnew, sizeof(Pnew));
}

/* Shared vector-measurement update.
   Both the accelerometer and magnetometer corrections have identical
   structure: compare a measured body-frame direction against a known
   navigation-frame reference rotated into the body frame. */
static int vector_update(ekf_t *f, const float meas[3],
                         const float ref_nav[3], float noise)
{
    float z[3], R[9], vb[3], y[3];
    float H[3*N];          /* 3 x 6 */
    float PHt[N*3];        /* 6 x 3 */
    float S[9], Sinv[9];
    float K[N*3];          /* 6 x 3 */
    float dx[N];
    float dq[4], qn[4];
    int i, j, k;

    if (!normalize3(meas, z)) return 0;

    quat_to_R(f->q, R);
    R_transpose_mul(R, ref_nav, vb);

    /* H = [ [vb x] | 0_3x3 ] */
    memset(H, 0, sizeof(H));
    H[0*N + 1] = -vb[2];  H[0*N + 2] =  vb[1];
    H[1*N + 0] =  vb[2];  H[1*N + 2] = -vb[0];
    H[2*N + 0] = -vb[1];  H[2*N + 1] =  vb[0];

    /* PHt = P * H^T   (6x6 * 6x3) */
    for (i = 0; i < N; i++)
        for (j = 0; j < 3; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) s += f->P[i*N + k] * H[j*N + k];
            PHt[i*3 + j] = s;
        }

    /* S = H * PHt + noise*I   (3x3) */
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) s += H[i*N + k] * PHt[k*3 + j];
            S[i*3 + j] = s + (i == j ? noise : 0.0f);
        }

    if (!inv3x3(S, Sinv)) return 0;

    /* K = PHt * Sinv   (6x3) */
    for (i = 0; i < N; i++)
        for (j = 0; j < 3; j++) {
            float s = 0.0f;
            for (k = 0; k < 3; k++) s += PHt[i*3 + k] * Sinv[k*3 + j];
            K[i*3 + j] = s;
        }

    /* Innovation and state correction */
    for (i = 0; i < 3; i++) y[i] = z[i] - vb[i];

    for (i = 0; i < N; i++) {
        float s = 0.0f;
        for (k = 0; k < 3; k++) s += K[i*3 + k] * y[k];
        dx[i] = s;
    }

    /* Apply the attitude error multiplicatively */
    dq[0] = 1.0f;
    dq[1] = 0.5f * dx[0];
    dq[2] = 0.5f * dx[1];
    dq[3] = 0.5f * dx[2];
    quat_mul(f->q, dq, qn);
    memcpy(f->q, qn, sizeof(qn));
    quat_normalize(f->q);

    f->bias[0] += dx[3];
    f->bias[1] += dx[4];
    f->bias[2] += dx[5];

    /* P -= K * (H P).  Since P is symmetric, H P = (P H^T)^T = PHt^T. */
    for (i = 0; i < N; i++)
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < 3; k++) s += K[i*3 + k] * PHt[j*3 + k];
            f->P[i*N + j] -= s;
        }

    /* Re-symmetrise to suppress accumulated rounding asymmetry */
    for (i = 0; i < N; i++)
        for (j = i + 1; j < N; j++) {
            float m = 0.5f * (f->P[i*N + j] + f->P[j*N + i]);
            f->P[i*N + j] = m;
            f->P[j*N + i] = m;
        }

    return 1;
}

int ekf_update_accel(ekf_t *f, const float acc[3])
{
    static const float g_nav[3] = { 0.0f, 0.0f, 1.0f };
    float mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

    /* Reject the update when the sample is not dominated by gravity.
       Under linear acceleration the measured vector no longer points
       along g, and using it would corrupt the attitude estimate. */
    if (mag < 0.85f || mag > 1.15f) return 0;

    return vector_update(f, acc, g_nav, f->cfg.accel_noise);
}

int ekf_update_mag(ekf_t *f, const float mag[3])
{
    return vector_update(f, mag, f->cfg.mag_ref, f->cfg.mag_noise);
}

void ekf_euler(const ekf_t *f, float *roll, float *pitch, float *yaw)
{
    float w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];
    float sinp;

    if (roll)
        *roll = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    if (pitch) {
        sinp = 2.0f*(w*y - z*x);
        if (sinp >  1.0f) sinp =  1.0f;
        if (sinp < -1.0f) sinp = -1.0f;
        *pitch = asinf(sinp);
    }

    if (yaw)
        *yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}
