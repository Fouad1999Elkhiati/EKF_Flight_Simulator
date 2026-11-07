#include "ekf_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
constexpr float kG = 9.81f;
constexpr float kDeg2Rad = 0.017453292f;  // DEG2RAD in imu_port.c
constexpr float kRad2Deg = 57.29578f;
// TM MPU9250 driver: MMult = 10 * 4912 / 32768 -> AK8963 counts to 0.1 uT (imu_port.c comment)
constexpr float kMMult = 10.0f * 4912.0f / 32768.0f;
constexpr float kAlignDt = 0.01f;         // 100 Hz, same loop rate as main.c / the sim
}  // namespace

namespace frames
{
glm::quat sim_to_flu(const glm::quat& q_sim)
{
  // M maps sim axes to FLU axes; columns are the images of the sim basis vectors.
  static const glm::quat qM = glm::quat_cast(glm::mat3(glm::vec3(1, 0, 0),    // sim x -> FLU x
                                                       glm::vec3(0, 0, 1),    // sim y (up) -> FLU z
                                                       glm::vec3(0, -1, 0)))  // sim z (right) -> FLU -y
      ;
  return glm::normalize(qM * q_sim * glm::conjugate(qM));
}

void euler_aircraft(const glm::quat& q, float& roll, float& pitch, float& yaw)
{
  // Same formulas as ekf_euler() in ekf.c (ZYX on the FLU frame) ...
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  const float r = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  const float sp = std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f);
  const float p = std::asin(sp);
  const float yw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  // ... then flipped to aircraft sense: FLU +pitch is nose-down, FLU +yaw is nose-left.
  roll = r;
  pitch = -p;
  yaw = -yw;
}
}  // namespace frames

EkfAdapter::EkfAdapter()
{
  // Inclined field (~60 deg dip), pointing north (+x) and down -- typical mid-latitude.
  mag_ref_flu = glm::normalize(glm::vec3(0.5f, 0.0f, -0.866f));
  reset();
}

glm::vec3 EkfAdapter::nrand() { return {n01_(rng_), n01_(rng_), n01_(rng_)}; }

// ADC quantisation + saturation of a 16-bit MPU9250 channel
float EkfAdapter::quantise(float v, float lsb_per_unit) const
{
  const float counts = std::clamp(std::round(v * lsb_per_unit), -32768.0f, 32767.0f);
  return counts / lsb_per_unit;
}

// ======================================================================================
// imu_port.c
// ======================================================================================

// read_mag_raw(): the AK8963 sits inside the MPU9250 with different axes,
//   body X = mag Y, body Y = mag X, body Z = -mag Z,
// and the driver converts counts to 0.1 uT with MMult. The board's hard/soft iron is applied
// in body axes, i.e. it travels with the airframe, which is what magcal models.
void EkfAdapter::read_mag_raw(const glm::vec3& b_body_unit, float out[3])
{
  glm::vec3 b_ut = b_body_unit * mag_env.field_ut;                    // true field, body, uT
  b_ut = mag_env.soft_iron * b_ut + mag_env.hard_iron_ut;               // board distortion
  b_ut += nrand() * (noise.mag_sigma * mag_env.field_ut);               // sensor noise

  // body -> AK8963 axes (inverse of the remap below)
  const float ak_ut[3] = {b_ut.y, b_ut.x, -b_ut.z};
  float raw_counts[3];
  for (int i = 0; i < 3; i++)
    raw_counts[i] = std::clamp(std::round(ak_ut[i] * 10.0f / kMMult), -32760.0f, 32760.0f);  // Mx_Raw...

  // --- verbatim from imu_port.c read_mag_raw() ---
  out[0] = raw_counts[1] * kMMult;    /* swap X <-> Y */
  out[1] = raw_counts[0] * kMMult;
  out[2] = -raw_counts[2] * kMMult;   /* negate Z */
}

int EkfAdapter::imu_port_read(const glm::quat& q_flu, const glm::vec3& w_flu, const glm::vec3& f_body_g, float dt,
                              PortSample& s)
{
  // The AK8963 converts on its own clock; a conversion stays latched (DRDY) until it is read.
  pending_dt_ += dt;
  mag_phase_ += dt;
  const float mag_period = 1.0f / std::max(noise.mag_rate_hz, 0.1f);
  bool mag_ready = false;
  if (mag_phase_ >= mag_period) {
    mag_phase_ = std::fmod(mag_phase_, mag_period);
    mag_ready = true;
  }

  // Accel and gyro are both required for a usable sample. On failure leave s untouched and do
  // not advance last_ms, so the next good sample's dt spans the whole gap.
  if (noise.read_fail_prob > 0.0f && u01_(rng_) < noise.read_fail_prob) {
    fail_total_++;
    if (mag_ready) mag_phase_ = mag_period;  // conversion still latched, read next time
    return 0;
  }

  // Library scales gyro to deg/s and accel to g; imu_port converts gyro to rad/s.
  const glm::vec3 gyro_dps = (w_flu + noise.gyro_bias + nrand() * noise.gyro_sigma) * kRad2Deg;
  const glm::vec3 acc_g = f_body_g + nrand() * noise.accel_sigma;
  for (int i = 0; i < 3; i++) {
    s.gyro[i] = quantise(gyro_dps[i], noise.gyro_lsb_per_dps) * kDeg2Rad;
    s.acc[i] = quantise(acc_g[i], noise.accel_lsb_per_g);
  }

  // mag: a fresh conversion only every 1/8 s, otherwise the driver's last raw values
  if (mag_ready) read_mag_raw(glm::inverse(q_flu) * mag_ref_flu, mag_last_);
  if (cal_.valid) {
    magcal_apply(&cal_, mag_last_, s.mag);
    s.mag_valid = 1;
  } else {
    s.mag[0] = mag_last_[0];
    s.mag[1] = mag_last_[1];
    s.mag[2] = mag_last_[2];
    s.mag_valid = 0;
  }
  s.mag_new = mag_ready ? 1 : 0;

  s.dt = pending_dt_;
  pending_dt_ = 0.0f;

  // Reject any implausible gap.
  return (s.dt > 0.0f && s.dt < 0.5f);
}

void EkfAdapter::imu_port_cal_begin() { magcal_reset(&cal_); }

// One fresh AK8963 sample folded into the fit while the airframe is being rotated.
int EkfAdapter::imu_port_cal_feed(const glm::vec3& b_body_unit)
{
  float raw[3];
  read_mag_raw(b_body_unit, raw);
  magcal_add(&cal_, raw[0], raw[1], raw[2]);
  return static_cast<int>(cal_.count);
}

int EkfAdapter::imu_port_cal_finish() { return magcal_solve(&cal_); }

// ======================================================================================
// main.c loop body
// ======================================================================================

void EkfAdapter::filter(const PortSample& s, const glm::quat& q_true, const glm::vec3& w_true_flu, float airspeed,
                        bool in_flight, ImuSample& imu, EkfState& out)
{
  ekf_predict(&ekf_, s.gyro, s.dt);

  // Remove the aircraft's own acceleration from the accel sample (fixed-wing compensation).
  // Body-frame kinematic acceleration with v_body ~ (V, 0, 0) along the nose:
  //   a = dV/dt * x  +  omega x (V, 0, 0)  =  (dV/dt,  wz*V,  -wy*V)   [FLU]
  float acc_for_ekf[3] = {s.acc[0], s.acc[1], s.acc[2]};
  if (prev_airspeed_ >= 0.0f && s.dt > 0.0f) {
    const float raw = (airspeed - prev_airspeed_) / s.dt;
    airspeed_dot_ += (raw - airspeed_dot_) * 0.2f;  // light low-pass, the derivative is noisy
  }
  prev_airspeed_ = airspeed;
  if (accel_compensation) {
    const float wy = s.gyro[1] - ekf_.bias[1];
    const float wz = s.gyro[2] - ekf_.bias[2];
    acc_for_ekf[0] -= airspeed_dot_ / kG;
    acc_for_ekf[1] -= wz * airspeed / kG;
    acc_for_ekf[2] -= -wy * airspeed / kG;
  }

  /* Returns 0 and skips the update when the accelerometer is not dominated by gravity. */
  out.accel_used = static_cast<uint32_t>(ekf_update_accel(&ekf_, acc_for_ekf));

  /* EKF_USE_MAG: only after the magnetometer has been calibrated, and only when the AK8963
     delivered a new reading -- re-applying the same one would over-weight the heading. */
  if (use_mag && s.mag_valid && s.mag_new) ekf_update_mag(&ekf_, s.mag);

  // ---------------- publish ----------------
  const glm::quat q_est(ekf_.q[0], ekf_.q[1], ekf_.q[2], ekf_.q[3]);
  for (int i = 0; i < 4; i++) out.q[i] = ekf_.q[i];
  for (int i = 0; i < 3; i++) out.bias[i] = ekf_.bias[i];
  ekf_euler(&ekf_, &out.roll, &out.pitch, &out.yaw);
  out.pitch = -out.pitch;  // aircraft sense: FLU +pitch is nose-down, FLU +yaw is nose-left
  out.yaw = -out.yaw;

  const float d = std::clamp(std::abs(glm::dot(q_est, q_true)), 0.0f, 1.0f);
  out.att_error_deg = glm::degrees(2.0f * std::acos(d));

  // what the rate loop sees: bias-corrected gyro, minus the truth it is standing in for
  const glm::vec3 w_meas(s.gyro[0] - ekf_.bias[0], s.gyro[1] - ekf_.bias[1], s.gyro[2] - ekf_.bias[2]);
  rate_err_sim_ = frames::flu_to_sim(w_meas - w_true_flu);

  for (int i = 0; i < 3; i++) {
    imu.gyro[i] = s.gyro[i];
    imu.accel[i] = s.acc[i];
    imu.mag[i] = s.mag[i];
  }
  imu.mag_new = static_cast<uint32_t>(s.mag_new);

  est_ = out;

  if (in_flight) {
    err_sum_ += out.att_error_deg;
    err_n_++;
    if (out.accel_used) acc_used_++;
    err_max_ = std::max(err_max_, out.att_error_deg);
  }
}

int EkfAdapter::step(const glm::quat& q_sim, const glm::vec3& w_body_sim, const glm::vec3& a_world, float airspeed,
                     float dt, uint64_t t_us, ImuSample& imu, EkfState& out)
{
  const glm::quat q_true = frames::sim_to_flu(q_sim);
  const glm::vec3 w_flu = frames::sim_to_flu(w_body_sim);
  // specific force = R^T (a - g_vec); g_vec = (0,-g,0) in the y-up sim world
  const glm::vec3 f_body_sim = glm::inverse(q_sim) * (a_world + glm::vec3(0.0f, kG, 0.0f));
  const glm::vec3 f_body_g = frames::sim_to_flu(f_body_sim) / kG;

  PortSample s{};
  if (!imu_port_read(q_true, w_flu, f_body_g, dt, s)) {
    out = est_;  // nothing new; the flight software keeps using the last estimate
    return 0;
  }
  imu.timestamp_us = t_us;
  filter(s, q_true, w_flu, airspeed, true, imu, out);
  return 1;
}

// ======================================================================================
// Pre-flight: ekf_init, magnetometer calibration sweep, stationary alignment
// ======================================================================================

void EkfAdapter::reset(const glm::quat& q_sim_launch)
{
  // Same tuning as the STM32 main.c
  cfg_.gyro_noise = 0.01f;
  cfg_.bias_noise = 0.0005f;
  cfg_.accel_noise = 0.05f;
  cfg_.mag_noise = 0.10f;
  cfg_.mag_ref[0] = mag_ref_flu.x;
  cfg_.mag_ref[1] = mag_ref_flu.y;
  cfg_.mag_ref[2] = mag_ref_flu.z;
  ekf_init(&ekf_, &cfg_);

  mag_phase_ = 0.0f;
  pending_dt_ = 0.0f;
  fail_total_ = 0;
  prev_airspeed_ = -1.0f;
  airspeed_dot_ = 0.0f;
  rate_err_sim_ = glm::vec3(0.0f);
  est_ = EkfState{};
  est_.q[0] = 1.0f;
  cal_offset_err_ = cal_scale_err_ = cal_norm_rms_ = 0.0f;

  // ---- 1. magnetometer calibration: imu_port_cal_begin / feed / finish ----
  // The airframe is rotated by hand through all orientations; the body-frame field direction
  // then sweeps the whole sphere (Fibonacci sampling of that sweep).
  imu_port_cal_begin();
  if (run_magcal) {
    const int n = std::max(magcal_samples, 1);
    const float golden = 2.39996323f;  // pi * (3 - sqrt(5))
    for (int i = 0; i < n; i++) {
      const float z = 1.0f - 2.0f * (i + 0.5f) / n;
      const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
      imu_port_cal_feed(glm::vec3(r * std::cos(golden * i), r * std::sin(golden * i), z));
    }
    if (imu_port_cal_finish()) {
      // compare with the distortion we injected (body axes, 0.1 uT)
      float off_err2 = 0.0f;
      for (int i = 0; i < 3; i++) {
        const float off_true = mag_env.hard_iron_ut[i] * 10.0f;
        const float scale_true = 1.0f / (mag_env.soft_iron[i] * mag_env.field_ut * 10.0f);
        off_err2 += (cal_.offset[i] - off_true) * (cal_.offset[i] - off_true);
        cal_scale_err_ = std::max(cal_scale_err_, 100.0f * std::abs(cal_.scale[i] / scale_true - 1.0f));
      }
      cal_offset_err_ = std::sqrt(off_err2);
      double sum = 0.0;
      const int m = 200;
      for (int i = 0; i < m; i++) {
        glm::vec3 d = nrand();
        d = glm::length(d) > 1e-3f ? glm::normalize(d) : glm::vec3(1, 0, 0);
        float raw[3], c[3];
        read_mag_raw(d, raw);
        magcal_apply(&cal_, raw, c);
        const double e = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) - 1.0;
        sum += e * e;
      }
      cal_norm_rms_ = static_cast<float>(std::sqrt(sum / m));
    }
  }

  // ---- 2. stationary alignment on the launch rail ----
  // Aircraft held still at the launch attitude: zero rate, specific force = +1 g up.
  const glm::quat q_true = frames::sim_to_flu(q_sim_launch);
  const glm::vec3 f_still = glm::inverse(q_true) * glm::vec3(0.0f, 0.0f, 1.0f);
  const int n_align = static_cast<int>(std::lround(std::max(align_s, 0.0f) / kAlignDt));
  for (int k = 0; k < n_align; k++) {
    PortSample s{};
    ImuSample imu{};
    EkfState out{};
    if (imu_port_read(q_true, glm::vec3(0.0f), f_still, kAlignDt, s))
      filter(s, q_true, glm::vec3(0.0f), 0.0f, false, imu, out);
  }
  if (n_align == 0) {  // no alignment: publish the initial (level) state
    ekf_euler(&ekf_, &est_.roll, &est_.pitch, &est_.yaw);
    const glm::quat q_est(ekf_.q[0], ekf_.q[1], ekf_.q[2], ekf_.q[3]);
    est_.att_error_deg = glm::degrees(2.0f * std::acos(std::clamp(std::abs(glm::dot(q_est, q_true)), 0.0f, 1.0f)));
  }

  // ---- 3. launch: airspeed derivative must not see the 0 -> cruise jump ----
  prev_airspeed_ = -1.0f;
  airspeed_dot_ = 0.0f;
  reset_stats();
}
