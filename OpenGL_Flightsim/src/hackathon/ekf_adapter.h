#pragma once
// The complete STM32 attitude stack from stm32_utilities/43-STM32Fxxx_MPU9250 _EKF/User,
// running inside the simulator. The algorithm files are compiled UNMODIFIED:
//
//   ekf.c / ekf.h                 6-state multiplicative EKF (attitude error + gyro bias)
//   magcal.c / magcal.h           axis-aligned ellipsoid fit (hard + soft iron)
//   Gauss_elimination.c / .h      Gauss-Jordan solver used by magcal_solve
//
// This file replaces the two hardware-facing files of the firmware:
//
//   imu_port.c  ->  EkfAdapter::imu_port_read / imu_port_cal_*
//       A simulated MPU9250 + AK8963. Gyro and accel are quantised to the sensor's LSB and
//       saturate at full scale. The magnetometer is produced in the AK8963's own axes, in
//       raw counts, with a hard-iron offset and soft-iron gains, at 8 Hz, and then remapped
//       exactly like read_mag_raw() (body X = mag Y, body Y = mag X, body Z = -mag Z) and
//       scaled by MMult to 0.1 uT. magcal_apply() runs on it once a calibration is solved,
//       mag_new is set only on a fresh AK8963 sample, and dt is gated to (0, 0.5) s.
//       Optional I2C read failures leave the sample untouched and let dt span the gap.
//
//   main.c      ->  EkfAdapter::step
//       ekf_predict every sample, ekf_update_accel every sample (|a| ~ 1 g gate inside),
//       ekf_update_mag only when mag_new, only if EKF_USE_MAG and the calibration is valid.
//
// plus the procedure an operator follows before a flight (EkfAdapter::reset):
//   1. magnetometer calibration sweep: imu_port_cal_begin / feed / finish while the airframe
//      is rotated through all orientations on the ground;
//   2. stationary alignment on the launch rail: the filter runs for align_s seconds with the
//      aircraft still, so roll/pitch lock to gravity and the gyro bias converges;
//   3. launch.
//
// The adapter also converts between the simulator's y-up frame and the EKF's z-up FLU frame,
// and exposes the bias-corrected gyro so the sim's rate loop can be closed on the sensor.

#include <cstdint>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "ekf.h"
#include "magcal.h"
#include "sim_types.h"

namespace frames
{
// sim (x fwd, y up, z right)  ->  FLU (x fwd, y left, z up)
inline glm::vec3 sim_to_flu(const glm::vec3& v) { return {v.x, -v.z, v.y}; }
inline glm::vec3 flu_to_sim(const glm::vec3& v) { return {v.x, v.z, -v.y}; }
glm::quat sim_to_flu(const glm::quat& q_sim);  // body->world (sim)  ->  body->nav (FLU)
// aircraft-style euler from a FLU body->nav quaternion: roll right +, pitch up +, yaw right +
void euler_aircraft(const glm::quat& q_flu, float& roll, float& pitch, float& yaw);
}  // namespace frames

// Simulated MPU9250 (gyro + accel) and AK8963 (magnetometer)
struct ImuNoise {
  float gyro_sigma = 0.005f;                     // rad/s per sample
  glm::vec3 gyro_bias{0.02f, -0.015f, 0.01f};    // rad/s (~1 deg/s), the EKF must find this
  float accel_sigma = 0.01f;                     // g
  float mag_sigma = 0.02f;                       // relative to |B| (per axis)
  float mag_rate_hz = 8.0f;                      // AK8963 continuous mode 1 (imu_port.h: 8 Hz)
  float gyro_lsb_per_dps = 65.5f;                // +/-500 dps full scale
  float accel_lsb_per_g = 8192.0f;               // +/-4 g full scale
  float read_fail_prob = 0.0f;                   // probability that an I2C read fails
};

// Board magnetic environment. Units of the AK8963 output after MMult: 0.1 uT.
struct MagEnvironment {
  float field_ut = 48.0f;                        // local field strength, uT
  glm::vec3 hard_iron_ut{12.0f, -8.0f, 20.0f};   // additive offset, body (FLU) axes, uT
  glm::vec3 soft_iron{1.10f, 0.92f, 1.04f};      // per-axis gain, body (FLU) axes
};

class EkfAdapter
{
 public:
  EkfAdapter();

  // ekf_init + the full pre-flight sequence (magcal sweep, stationary alignment) at the launch
  // attitude q_sim_launch. Call after FixedWing::reset.
  void reset(const glm::quat& q_sim_launch = glm::quat(1, 0, 0, 0));

  // One 100 Hz loop of the firmware: imu_port_read() on the true state, then the main.c
  // sequence. Returns 0 when imu_port_read rejected the sample (I2C failure or bad dt);
  // the filter is then unchanged and `ekf_out` holds the previous estimate.
  // q_sim: body->world, w_body_sim: body rates, a_world: kinematic acceleration,
  // airspeed: what a pitot tube would read (m/s), used for accel compensation.
  int step(const glm::quat& q_sim, const glm::vec3& w_body_sim, const glm::vec3& a_world, float airspeed,
           float dt, uint64_t t_us, ImuSample& imu_out, EkfState& ekf_out);

  // ---- what the flight control software reads ----
  const EkfState& estimate() const { return est_; }
  // (gyro - estimated bias) - true rate, in sim body axes. The rate loop regulates the measured
  // rate, so the aircraft really rotates at (command - rate_error_sim).
  const glm::vec3& rate_error_sim() const { return rate_err_sim_; }

  // ---- configuration (applied at the next reset unless noted) ----
  ImuNoise noise;
  MagEnvironment mag_env;
  glm::vec3 mag_ref_flu;            // nav-frame magnetic reference (unit), what cfg.mag_ref holds
  bool use_mag = true;              // EKF_USE_MAG in main.c (takes effect immediately)
  bool run_magcal = true;           // do the calibration sweep before flight
  int magcal_samples = 400;         // fresh AK8963 samples collected during the sweep (~50 s at 8 Hz)
  float align_s = 10.0f;            // stationary alignment time on the launch rail, s (0 = skip)

  // Fixed-wing accelerometer compensation (done in the "flight software" layer, ekf.c stays
  // unmodified). In a turn the accelerometer reads gravity + centripetal acceleration, so the
  // plain filter is pulled back towards wings-level. With an airspeed sensor the centripetal
  // part is omega x V and the longitudinal part is dV/dt; both are removed before the accel
  // update. Toggle with the C key in the sim to compare the EKF with and without it.
  bool accel_compensation = true;

  // ---- diagnostics ----
  bool mag_cal_valid() const { return cal_.valid != 0; }
  const magcal_t& magcal() const { return cal_; }
  float magcal_offset_err() const { return cal_offset_err_; }   // |offset - truth|, 0.1 uT
  float magcal_scale_err_pct() const { return cal_scale_err_; } // max per-axis scale error, %
  float magcal_norm_rms() const { return cal_norm_rms_; }       // RMS of (|calibrated| - 1)
  uint32_t read_errors() const { return fail_total_; }
  bool mag_in_use() const { return use_mag && cal_.valid; }

  // Running statistics of the attitude error since the last reset (in flight only).
  void reset_stats() { err_sum_ = 0.0; err_n_ = acc_used_ = 0; err_max_ = 0.0f; }
  float err_mean_deg() const { return err_n_ ? static_cast<float>(err_sum_ / err_n_) : 0.0f; }
  float err_max_deg() const { return err_max_; }
  float accel_used_pct() const { return err_n_ ? 100.0f * static_cast<float>(acc_used_) / err_n_ : 0.0f; }

 private:
  // ---- imu_port.c equivalent ----
  struct PortSample {     // imu_sample_t from imu_port.h
    float gyro[3];        // rad/s
    float acc[3];         // g
    float mag[3];         // calibrated unit vector, or raw 0.1 uT if uncalibrated
    float dt;             // s
    int mag_valid;
    int mag_new;
  };
  int imu_port_read(const glm::quat& q_flu, const glm::vec3& w_flu, const glm::vec3& f_body_flu_g, float dt,
                    PortSample& s);
  void imu_port_cal_begin();
  int imu_port_cal_feed(const glm::vec3& b_body_unit);
  int imu_port_cal_finish();
  void read_mag_raw(const glm::vec3& b_body_unit, float out[3]);  // AK8963 axes + remap + MMult

  // ---- main.c loop body + compensation + stats ----
  void filter(const PortSample& s, const glm::quat& q_true, const glm::vec3& w_true_flu, float airspeed,
              bool in_flight, ImuSample& imu, EkfState& out);

  float quantise(float v, float lsb_per_unit) const;
  glm::vec3 nrand();

  ekf_t ekf_{};
  ekf_cfg_t cfg_{};
  magcal_t cal_{};
  EkfState est_{};
  glm::vec3 rate_err_sim_{0.0f};
  std::mt19937 rng_{1234};
  std::normal_distribution<float> n01_{0.0f, 1.0f};
  std::uniform_real_distribution<float> u01_{0.0f, 1.0f};

  float mag_phase_ = 0.0f;       // time since the last AK8963 conversion
  float pending_dt_ = 0.0f;      // dt accumulated across failed reads (last_ms not advanced)
  float mag_last_[3] = {0, 0, 0};
  uint32_t fail_total_ = 0;

  float prev_airspeed_ = -1.0f;
  float airspeed_dot_ = 0.0f;

  float cal_offset_err_ = 0.0f, cal_scale_err_ = 0.0f, cal_norm_rms_ = 0.0f;

  double err_sum_ = 0.0;
  uint64_t err_n_ = 0, acc_used_ = 0;
  float err_max_ = 0.0f;
};
