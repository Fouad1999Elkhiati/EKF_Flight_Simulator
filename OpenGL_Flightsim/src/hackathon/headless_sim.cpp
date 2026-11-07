// Headless test harness for the hackathon UAV mode.
//
// Runs exactly the same FixedWing physics, EkfAdapter (your unmodified ekf.c) and
// SimBridge shared memory as the OpenGL simulator, but without a window: the onboard
// camera image is synthesised by projecting the target sphere into the camera and
// drawing it as a magenta disc on a sky/ground background. controller.py cannot tell
// the difference, so you can test the EKF + guidance loop on any machine, e.g. in CI.
//
//   build : cmake --build . --target flightsim_headless     (needs only glm)
//   run   : ./flightsim_headless [--time S] [--speedup X] [--no-comp] [--inner ekf|truth]
//                                [--align S] [--no-magcal] [--no-mag] [--i2c-fail P] [--mag-hz F]
//           python controller.py               (in a second terminal)
//
// Target placement and scoring are the same as uav_app.cpp.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <thread>

#include "ekf_adapter.h"
#include "fixed_wing.h"
#include "sim_bridge.h"

namespace
{
constexpr float kDt = 0.01f;
constexpr float kCamVfovDeg = 60.0f;
constexpr float kCamTiltDeg = 15.0f;
constexpr float kTargetRadius = 2.5f;
constexpr float kHitMargin = 2.0f;
constexpr float kLinkTimeout = 0.5f;

struct TargetSim {
  glm::vec3 pos{0.0f};
  std::mt19937 rng{42};
  void respawn(const glm::vec3& from, float heading, float distance, float spread, float dh)
  {
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    const float b = heading + u(rng) * spread;
    pos = from + glm::vec3(std::cos(b) * distance, u(rng) * dh, std::sin(b) * distance);
  }
};

glm::quat camera_rotation(const glm::quat& att)
{
  return att * glm::angleAxis(glm::radians(-90.0f), glm::vec3(0, 1, 0)) *
         glm::angleAxis(glm::radians(kCamTiltDeg), glm::vec3(1, 0, 0));
}

// Synthetic onboard camera, same projection as gfx::Camera (glm::perspective, -z forward)
void render(SharedBlock* blk, const FixedWing& fw, const glm::vec3& target)
{
  uint8_t* px = blk->pixels;
  const glm::quat qc = camera_rotation(fw.attitude);
  const glm::quat qci = glm::inverse(qc);
  const float f = 1.0f / std::tan(glm::radians(kCamVfovDeg) * 0.5f);
  const float aspect = static_cast<float>(CAM_WIDTH) / CAM_HEIGHT;

  // background: sky above the horizon, ground below (row 0 = bottom, OpenGL style)
  for (int y = 0; y < CAM_HEIGHT; y++) {
    for (int x = 0; x < CAM_WIDTH; x++) {
      const float nx = (2.0f * (x + 0.5f) / CAM_WIDTH - 1.0f) * aspect / f;
      const float ny = (2.0f * (y + 0.5f) / CAM_HEIGHT - 1.0f) / f;
      const glm::vec3 dir_world = qc * glm::vec3(nx, ny, -1.0f);
      uint8_t* p = px + (static_cast<size_t>(y) * CAM_WIDTH + x) * 3;
      if (dir_world.y > 0.0f) { p[0] = 170; p[1] = 210; p[2] = 240; }  // sky
      else                    { p[0] = 90;  p[1] = 120; p[2] = 70; }   // ground
    }
  }

  const glm::vec3 pc = qci * (target - fw.position);
  if (pc.z > -0.3f) return;  // behind the camera
  const float depth = -pc.z;
  const float sx = (f / aspect) * pc.x / depth;
  const float sy = f * pc.y / depth;
  const float cx = (sx + 1.0f) * 0.5f * CAM_WIDTH;
  const float cy = (sy + 1.0f) * 0.5f * CAM_HEIGHT;
  const float dist = glm::length(target - fw.position);
  const float ang = std::asin(std::min(1.0f, kTargetRadius / std::max(dist, kTargetRadius)));
  const float rpx = std::tan(ang) * f * 0.5f * CAM_HEIGHT;
  const int x0 = std::max(0, static_cast<int>(cx - rpx - 1)), x1 = std::min(CAM_WIDTH - 1, static_cast<int>(cx + rpx + 1));
  const int y0 = std::max(0, static_cast<int>(cy - rpx - 1)), y1 = std::min(CAM_HEIGHT - 1, static_cast<int>(cy + rpx + 1));
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++) {
      const float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
      if (dx * dx + dy * dy <= rpx * rpx) {
        uint8_t* p = px + (static_cast<size_t>(y) * CAM_WIDTH + x) * 3;
        p[0] = 255; p[1] = 0; p[2] = 255;
      }
    }
}
}  // namespace

int main(int argc, char** argv)
{
  float max_time = 0.0f, speedup = 1.0f;
  bool comp = true, inner_ekf = true, magcal = true, use_mag = true;
  float align_s = 10.0f, fail_p = 0.0f, mag_hz = 8.0f;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--time" && i + 1 < argc) max_time = std::stof(argv[++i]);
    else if (a == "--speedup" && i + 1 < argc) speedup = std::stof(argv[++i]);
    else if (a == "--no-comp") comp = false;
    else if (a == "--inner" && i + 1 < argc) inner_ekf = std::string(argv[++i]) != "truth";
    else if (a == "--align" && i + 1 < argc) align_s = std::stof(argv[++i]);
    else if (a == "--no-magcal") magcal = false;
    else if (a == "--no-mag") use_mag = false;
    else if (a == "--i2c-fail" && i + 1 < argc) fail_p = std::stof(argv[++i]);
    else if (a == "--mag-hz" && i + 1 < argc) mag_hz = std::stof(argv[++i]);
  }

  SimBridge bridge;
  if (!bridge.create()) return 1;
  SharedBlock* blk = bridge.data();
  blk->camera.vfov_deg = kCamVfovDeg;
  blk->camera.tilt_deg = kCamTiltDeg;

  const glm::vec3 start(0.0f, 3000.0f, 0.0f);
  FixedWing fw;
  EkfAdapter ekf;
  ekf.accel_compensation = comp;
  ekf.align_s = align_s;
  ekf.run_magcal = magcal;
  ekf.use_mag = use_mag;
  ekf.noise.read_fail_prob = fail_p;
  ekf.noise.mag_rate_hz = mag_hz;
  TargetSim tgt;

  auto reset = [&]() {
    fw.reset(start, 0.0f);
    ekf.reset(fw.attitude);  // ekf_init + magcal sweep + stationary alignment
    std::printf("[headless] preflight: magcal %s (offset err %.1f x0.1uT, scale err %.2f%%, |m| rms %.3f), "
                "aligned %.0f s: att err %.2f deg, bias err %.3f dps\n",
                ekf.mag_cal_valid() ? "OK" : "none", ekf.magcal_offset_err(), ekf.magcal_scale_err_pct(),
                ekf.magcal_norm_rms(), ekf.align_s, ekf.estimate().att_error_deg,
                glm::degrees(glm::length(glm::vec3(ekf.estimate().bias[0], ekf.estimate().bias[1],
                                                   ekf.estimate().bias[2]) - ekf.noise.gyro_bias)));
    std::fflush(stdout);
    tgt.pos = start + glm::vec3(300.0f, 20.0f, 0.0f);
    blk->target.hits = 0;
    blk->target.last_hit_step = 0;
  };
  reset();

  std::printf("[headless] running (inner loop on %s, accel compensation %s, mag %s, speedup %.1fx)\n",
              inner_ekf ? "EKF" : "TRUTH", comp ? "ON" : "OFF", use_mag ? "ON" : "OFF", speedup);
  std::fflush(stdout);

  uint64_t steps = 0, t_us = 0, last_seq = 0;
  float sim_since_cmd = 1e9f;
  bool link_was = false;
  float speed_int = 0.0f;
  auto wall0 = std::chrono::steady_clock::now();

  while (max_time <= 0.0f || t_us * 1e-6f < max_time) {
    ControlCmd cmd = blk->cmd;
    if (cmd.reset_request) {
      blk->cmd.reset_request = 0;
      reset();
    }
    if (cmd.seq != last_seq) { last_seq = cmd.seq; sim_since_cmd = 0.0f; }
    else sim_since_cmd += kDt;
    const bool link = cmd.armed && last_seq != 0 && sim_since_cmd < kLinkTimeout * speedup;
    if (link != link_was) {
      std::printf(link ? "[headless] controller connected -- AUTO\n" : "[headless] link lost -- FAILSAFE\n");
      std::fflush(stdout);
      link_was = link;
    }

    float rc = 0, pc = 0, spd = FixedWing::cruise_speed();
    if (link) {
      rc = std::clamp(cmd.roll_angle, -glm::radians(45.0f), glm::radians(45.0f));
      pc = std::clamp(cmd.pitch_angle, -glm::radians(25.0f), glm::radians(25.0f));
      if (cmd.target_speed > 1.0f) spd = cmd.target_speed;
    }
    const float e = spd - fw.airspeed();
    speed_int = std::clamp(speed_int + e * kDt, -5.0f, 5.0f);
    const float thr = std::clamp(FixedWing::cruise_throttle() + 0.05f * e + 0.01f * speed_int, 0.0f, 1.0f);

    // inner loop on the last EKF output (same as uav_app.cpp)
    FixedWing::InnerLoopFeedback fb;
    fb.roll = ekf.estimate().roll;
    fb.pitch = ekf.estimate().pitch;
    fb.rate_error = ekf.rate_error_sim();
    fw.step(rc, pc, thr, kDt, inner_ekf ? &fb : nullptr);
    t_us += 10000;
    steps++;

    ImuSample imu = blk->imu;  // kept if the simulated I2C read fails
    EkfState est{};
    ekf.step(fw.attitude, fw.ang_vel, fw.accel_world, fw.airspeed(), kDt, t_us, imu, est);

    const float dist = glm::length(fw.position - tgt.pos) - kTargetRadius;
    if (dist < kHitMargin) {
      blk->target.hits++;
      blk->target.last_hit_step = static_cast<uint32_t>(steps);
      std::printf("[headless] TARGET HIT #%u at t=%.2f s  (EKF err mean %.2f, max %.2f deg)\n", blk->target.hits,
                  t_us * 1e-6, ekf.err_mean_deg(), ekf.err_max_deg());
      std::fflush(stdout);
      float r, p, h;
      frames::euler_aircraft(frames::sim_to_flu(fw.attitude), r, p, h);
      tgt.respawn(fw.position, h, 250.0f, glm::radians(45.0f), 25.0f);
    }

    // publish (same fields as UavApp::publish_truth)
    blk->imu = imu;
    blk->ekf = est;
    const glm::quat q = frames::sim_to_flu(fw.attitude);
    SimState& s = blk->truth;
    s.attitude_q[0] = q.w; s.attitude_q[1] = q.x; s.attitude_q[2] = q.y; s.attitude_q[3] = q.z;
    frames::euler_aircraft(q, s.roll, s.pitch, s.yaw);
    const glm::vec3 w = frames::sim_to_flu(fw.ang_vel);
    for (int i = 0; i < 3; i++) {
      s.ang_vel[i] = w[i];
      s.position[i] = fw.position[i];
      s.velocity[i] = fw.velocity[i];
      blk->target.position[i] = tgt.pos[i];
    }
    s.airspeed = fw.airspeed();
    s.altitude = fw.position.y;
    blk->target.radius = kTargetRadius;
    blk->target.distance = dist;
    blk->header.sim_time_us = t_us;
    blk->header.physics_steps = steps;

    if (steps % 3 == 0) {  // ~33 fps camera
      render(blk, fw, tgt.pos);
      blk->camera.frame_id++;
    }

    // real-time pacing (scaled)
    const auto due = wall0 + std::chrono::microseconds(static_cast<int64_t>(t_us / speedup));
    std::this_thread::sleep_until(due);
  }

  std::printf("[headless] done: %u hits in %.1f s, EKF err mean %.2f max %.2f deg, accel used %.0f%%, "
              "i2c errors %u\n",
              blk->target.hits, t_us * 1e-6, ekf.err_mean_deg(), ekf.err_max_deg(), ekf.accel_used_pct(),
              ekf.read_errors());
  return 0;
}
