#pragma once
// Minimal fixed-wing rigid-body model in the simulator's frames:
//   world: x forward/north, y UP, z right/east
//   body : x nose, y up, z right wing
//
// Inputs are *angle* commands (what a stabilization loop tracks) plus throttle.
// Internally there is a P-loop from Euler-angle error to Euler rates, converted to
// body rates (with turn coordination), then a first-order rate response.
//
// Aerodynamics (simple but consistent):
//   * lift  = qS * CL(alpha), perpendicular to the airflow, in the body x-y plane
//   * drag  = qS * (CD0 + k CL^2), opposite to the airflow
//   * side  = qS * CYb * beta, pushes the airflow back onto the nose (no skidding)
//   * coordinated turns: when banked the nose yaws at g*tan(phi)/V, so the nose,
//     the velocity vector and the forward camera stay aligned while turning.
//
// FIXES vs. the first version:
//   - the nose never yawed in a bank, so velocity turned while the nose/camera kept
//     pointing the old way -> the target drifted out of view and was never reached;
//   - lift did not depend on angle of attack, so pitch commands did not change the
//     flight path (only tilted the thrust) and the aircraft could fly sideways.

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "ekf_adapter.h"  // for frames::euler_aircraft / frames::sim_to_flu

class FixedWing
{
 public:
  // Physical constants tuned for a small UAV (~2 kg)
  static constexpr float kMass = 2.0f;           // kg
  static constexpr float kGravity = 9.81f;       // m/s^2
  static constexpr float kMaxThrust = 20.0f;     // N
  static constexpr float kQS = 0.2756f;          // 0.5 * rho * S  (rho 1.225, S 0.45 m^2)
  static constexpr float kCL0 = 0.20f;           // lift coefficient at alpha = 0
  static constexpr float kCLa = 5.0f;            // per rad
  static constexpr float kCLmax = 1.2f;
  static constexpr float kAlphaStall = 0.24f;    // rad (~14 deg)
  static constexpr float kCD0 = 0.05f;
  static constexpr float kInducedK = 0.06f;
  static constexpr float kCYb = 2.0f;            // side-force per rad of sideslip
  static constexpr float kMaxRollRate = 2.5f;    // rad/s (inner loop clamp)
  static constexpr float kMaxPitchRate = 1.0f;   // rad/s
  static constexpr float kMaxYawRate = 1.0f;     // rad/s
  static constexpr float kRateTau = 0.10f;       // s, rate-loop lag
  static constexpr float kRollKp = 3.0f;         // angle error -> Euler rate
  static constexpr float kPitchKp = 2.5f;

  glm::vec3 position{0.0f};
  glm::vec3 velocity{0.0f};
  glm::quat attitude{1.0f, 0.0f, 0.0f, 0.0f};  // body -> world (sim)
  glm::vec3 ang_vel{0.0f};                     // sim body axes: x=roll, y=yaw-left, z=pitch-up
  glm::vec3 accel_world{0.0f};                 // kinematic acceleration, m/s^2
  float alpha = 0.0f;                          // angle of attack, rad
  float beta = 0.0f;                           // sideslip, rad

  static constexpr float cruise_speed() { return 18.0f; }     // m/s
  static constexpr float cruise_throttle() { return 0.25f; }  // level flight at cruise

  void reset(const glm::vec3& start_pos, float heading_rad = 0.0f)
  {
    position = start_pos;
    // Fixed-wing must be moving to fly: start at cruise speed along heading
    velocity = glm::vec3(std::cos(heading_rad), 0.0f, std::sin(heading_rad)) * cruise_speed();
    // heading > 0 = nose right; in the sim's y-up frame that is a negative rotation about +y
    attitude = glm::angleAxis(-heading_rad, glm::vec3(0, 1, 0));
    ang_vel = glm::vec3(0.0f);
    accel_world = glm::vec3(0.0f);
    alpha = beta = 0.0f;
  }

  // What the stabilisation loops measure. With nullptr, step() uses the true state (ideal
  // sensors). With a feedback, the angle loop tracks the command against the *estimated*
  // roll/pitch and the rate loop regulates the *measured* (bias-corrected) gyro, so the
  // aircraft really flies on the EKF: an attitude error becomes a real bank/pitch error and
  // a residual gyro bias becomes a real rotation-rate error.
  struct InnerLoopFeedback {
    float roll = 0.0f;              // rad, aircraft convention (EKF estimate)
    float pitch = 0.0f;             // rad, aircraft convention (EKF estimate)
    glm::vec3 rate_error{0.0f};     // measured - true body rate, sim body axes (rad/s)
  };

  // roll_angle_cmd, pitch_angle_cmd in aircraft convention (rad), + = right / up.
  // throttle in [0, 1].
  void step(float roll_angle_cmd, float pitch_angle_cmd, float throttle, float dt,
            const InnerLoopFeedback* fb = nullptr)
  {
    const float speed = glm::length(velocity);

    // ---- current aircraft euler: truth, or what the autopilot believes ----
    float phi, theta, psi;
    frames::euler_aircraft(frames::sim_to_flu(attitude), phi, theta, psi);
    if (fb) {
      phi = fb->roll;
      theta = fb->pitch;
    }

    // ---- outer loop: angle error -> Euler rates ----
    const float phi_dot = std::clamp(kRollKp * (roll_angle_cmd - phi), -kMaxRollRate, kMaxRollRate);
    const float theta_dot = std::clamp(kPitchKp * (pitch_angle_cmd - theta), -kMaxPitchRate, kMaxPitchRate);
    // coordinated turn: heading rate that keeps the ball centred
    const float phi_c = std::clamp(phi, -1.2f, 1.2f);
    const float psi_dot =
        std::clamp(kGravity * std::tan(phi_c) / std::max(speed, 8.0f), -kMaxYawRate, kMaxYawRate);

    // ---- Euler rates -> aircraft body rates (p roll, q pitch-up, r yaw-right) ----
    const float sphi = std::sin(phi), cphi = std::cos(phi);
    const float sth = std::sin(theta), cth = std::cos(theta);
    const float p = phi_dot - psi_dot * sth;
    const float q = theta_dot * cphi + psi_dot * sphi * cth;
    const float r = -theta_dot * sphi + psi_dot * cphi * cth;

    // aircraft (p, q, r) -> sim body (x roll, y yaw-LEFT, z pitch-up)
    const glm::vec3 rate_cmd(p, -r, q);

    // ---- first-order rate loop ----
    // The loop drives the measured rate (true + rate_error) to the command, so the true
    // rate settles at command - rate_error.
    const glm::vec3 rate_target = fb ? rate_cmd - fb->rate_error : rate_cmd;
    const float a = 1.0f - std::exp(-dt / kRateTau);
    ang_vel += (rate_target - ang_vel) * a;

    // ---- integrate attitude ----
    const float angle = glm::length(ang_vel) * dt;
    if (angle > 1e-9f) {
      attitude = glm::normalize(attitude * glm::angleAxis(angle, glm::normalize(ang_vel)));
    }

    // ---- aerodynamics ----
    const glm::vec3 nose = attitude * glm::vec3(1, 0, 0);
    const glm::vec3 wing = attitude * glm::vec3(0, 0, 1);
    const glm::vec3 vel_dir = speed > 0.1f ? velocity / speed : nose;

    const glm::vec3 v_body = glm::inverse(attitude) * velocity;
    alpha = std::atan2(-v_body.y, std::max(v_body.x, 0.1f));
    beta = speed > 0.1f ? std::asin(std::clamp(v_body.z / speed, -1.0f, 1.0f)) : 0.0f;

    float CL = kCL0 + kCLa * alpha;
    if (std::abs(alpha) > kAlphaStall) CL *= 0.5f;  // stalled: lift collapses
    CL = std::clamp(CL, -kCLmax, kCLmax);

    const float qS = kQS * speed * speed;

    // lift perpendicular to the airflow, in the plane of nose and body-up
    glm::vec3 lift_dir = glm::cross(wing, vel_dir);
    const float ll = glm::length(lift_dir);
    lift_dir = ll > 1e-3f ? lift_dir / ll : attitude * glm::vec3(0, 1, 0);

    const glm::vec3 lift = lift_dir * (qS * CL);
    const glm::vec3 drag = -vel_dir * (qS * (kCD0 + kInducedK * CL * CL));
    const glm::vec3 side = -wing * (qS * kCYb * beta);
    const glm::vec3 thrust = nose * (std::clamp(throttle, 0.0f, 1.0f) * kMaxThrust);
    const glm::vec3 gravity(0, -kGravity * kMass, 0);

    accel_world = (thrust + lift + drag + side + gravity) / kMass;
    velocity += accel_world * dt;
    position += velocity * dt;

    // ground
    if (position.y < 0.5f) {
      position.y = 0.5f;
      if (velocity.y < 0) velocity.y = 0;
    }
  }

  float airspeed() const { return glm::length(velocity); }
};
