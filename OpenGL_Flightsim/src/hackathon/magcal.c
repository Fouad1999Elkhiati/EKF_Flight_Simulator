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
#include "magcal.h"
#include "Gauss_elimination.h"
#include <math.h>
#include <string.h>

#define NP 6                       /* number of fitted parameters */

/* Samples are divided by this before accumulation. Fourth-order terms
   grow fast: raw microtesla readings around 50 give x^4 near 6e6, and
   summing thousands of those loses precision even in double. Scaling
   to order 1 keeps the normal matrix well conditioned. */
#define MAG_SCALE  50.0f

void magcal_reset(magcal_t *c)
{
    memset(c, 0, sizeof(*c));
    c->scale[0] = c->scale[1] = c->scale[2] = 1.0f;
}

void magcal_add(magcal_t *c, float mx, float my, float mz)
{
    double row[NP];
    double target;
    double x = (double)(mx / MAG_SCALE);
    double y = (double)(my / MAG_SCALE);
    double z = (double)(mz / MAG_SCALE);
    int i, j;

    /* x^2 + B y^2 + C z^2 + P x + Q y + R z + D = 0
       rearranged as  [y^2 z^2 x y z 1] . v = -x^2   */
    row[0] = y * y;
    row[1] = z * z;
    row[2] = x;
    row[3] = y;
    row[4] = z;
    row[5] = 1.0;

    target = -(x * x);

    /* Fold directly into the normal equations. The design matrix itself
       is never stored, so memory is constant in the sample count. */
    for (i = 0; i < NP; i++) {
        for (j = 0; j < NP; j++)
            c->ata[i*NP + j] += row[i] * row[j];
        c->atb[i] += row[i] * target;
    }

    c->count++;
}

int magcal_solve(magcal_t *c)
{
    float A[NP*NP];
    float aug[NP*NP];
    float v[NP];
    float B, C, P, Q, R, D;
    float x0, y0, z0, K;
    int i, j;

    c->valid = 0;

    if (c->count < MAGCAL_MIN_SAMPLES) return 0;

    /* Down-convert to float for the solver. Accumulation was done in
       double to avoid cancellation across many samples; the solve
       itself is well conditioned enough for single precision. */
    for (i = 0; i < NP*NP; i++) A[i] = (float)c->ata[i];
    for (i = 0; i < NP;    i++) v[i] = (float)c->atb[i];

    /* determinant() consumes A and requires an augmented identity
       alongside; inversion() then completes the back substitution and
       leaves the solution in v. */
    for (i = 0; i < NP; i++)
        for (j = 0; j < NP; j++)
            aug[i*NP + j] = (i == j) ? 1.0f : 0.0f;

    if (determinant(A, aug, v, NP) == 0.0f) return 0;

    inversion(A, aug, v, NP);

    B = v[0]; C = v[1];
    P = v[2]; Q = v[3]; R = v[4]; D = v[5];

    /* A negative quadratic coefficient means the fitted surface is a
       hyperboloid, not an ellipsoid -- the samples did not cover enough
       orientations. */
    if (B <= 1e-6f || C <= 1e-6f) return 0;

    /* Complete the square on each axis to recover the centre. */
    x0 = -0.5f * P;
    y0 = -0.5f * Q / B;
    z0 = -0.5f * R / C;

    K = x0*x0 + B*y0*y0 + C*z0*z0 - D;

    if (K <= 1e-9f) return 0;

    c->offset[0] = x0 * MAG_SCALE;
    c->offset[1] = y0 * MAG_SCALE;
    c->offset[2] = z0 * MAG_SCALE;

    /* Semi-axis lengths are sqrt(K / coefficient). Dividing each axis
       by its own length maps the ellipsoid onto the unit sphere. */
    c->scale[0] = 1.0f / (sqrtf(K)         * MAG_SCALE);
    c->scale[1] = 1.0f / (sqrtf(K / B)     * MAG_SCALE);
    c->scale[2] = 1.0f / (sqrtf(K / C)     * MAG_SCALE);

    c->valid = 1;
    return 1;
}

void magcal_apply(const magcal_t *c, const float raw[3], float out[3])
{
    out[0] = (raw[0] - c->offset[0]) * c->scale[0];
    out[1] = (raw[1] - c->offset[1]) * c->scale[1];
    out[2] = (raw[2] - c->offset[2]) * c->scale[2];
}
