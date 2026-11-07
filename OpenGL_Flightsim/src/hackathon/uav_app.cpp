#include "uav_app.h"
#include <GL/glew.h>
#include <SDL.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "../../lib/imgui/imgui.h"

namespace {
constexpr float kPhysicsDt = 0.01f;
constexpr int kMaxStepsPerFrame = 10;
}

UavApp::UavApp() = default;

bool UavApp::init(gfx::Object3D& scene, int window_w, int window_h)
{
  win_w_ = window_w;
  win_h_ = window_h;

  if (!bridge_.create()) {
    std::fprintf(stderr, "[uav] shared memory bridge failed\n");
  } else {
    bridge_.data()->camera.vfov_deg = camera_vfov_deg;
    bridge_.data()->camera.tilt_deg = camera_tilt_deg;
  }

  target_ = std::make_unique<Target>(2.5f); // Slightly bigger target for the jet
  scene.add(&target_->mesh());

  // REMOVED the ugly cube generation. The jet will be attached externally.

  const float aspect = static_cast<float>(CAM_WIDTH) / static_cast<float>(CAM_HEIGHT);
  onboard_cam_ = std::make_unique<gfx::Camera>(glm::radians(camera_vfov_deg), aspect, 0.3f, 150000.0f);
  scene.add(onboard_cam_.get());

  reset();
  std::printf("[uav] FIXED-WING UAV mode: R reset, V toggle view, C EKF accel-comp on/off, E inner loop EKF/truth, "
              "M mag on/off, O orbit, P pause.\n");
  return true;
}

void UavApp::reset()
{
  fw_.reset(start_position, 0.0f);
  ekf_.reset(fw_.attitude);  // ekf_init + magcal sweep + stationary alignment at launch attitude
  std::printf("[uav] preflight: magcal %s (offset err %.1f x0.1uT, scale err %.2f%%, |m| rms %.3f), "
              "aligned %.0f s, bias est %.2f %.2f %.2f dps\n",
              ekf_.mag_cal_valid() ? "OK" : "FAILED", ekf_.magcal_offset_err(), ekf_.magcal_scale_err_pct(),
              ekf_.magcal_norm_rms(), ekf_.align_s, glm::degrees(ekf_.estimate().bias[0]),
              glm::degrees(ekf_.estimate().bias[1]), glm::degrees(ekf_.estimate().bias[2]));
  std::fflush(stdout);
  target_->set_position(start_position + glm::vec3(300.0f, 20.0f, 0.0f));
  accumulator_ = 0.0f;
  speed_integral_ = 0.0f;
  if (bridge_.ok()) {
    bridge_.data()->target.hits = 0;
    bridge_.data()->target.last_hit_step = 0;
  }
  publish_truth();
}

void UavApp::handle_key(int key)
{
  if (key == SDLK_r) reset();
  if (key == SDLK_v) view_ = (view_ == View::Chase) ? View::Onboard : View::Chase;
  if (key == SDLK_c) {
    ekf_.accel_compensation = !ekf_.accel_compensation;
    ekf_.reset_stats();  // restart the statistics so the two modes can be compared
    std::printf("[uav] EKF accel compensation %s\n", ekf_.accel_compensation ? "ON" : "OFF");
  }
  if (key == SDLK_e) {
    inner_loop_on_ekf = !inner_loop_on_ekf;
    ekf_.reset_stats();
    std::printf("[uav] inner loop flies on %s\n", inner_loop_on_ekf ? "EKF estimate" : "TRUE attitude");
  }
  if (key == SDLK_m) {
    ekf_.use_mag = !ekf_.use_mag;  // EKF_USE_MAG
    ekf_.reset_stats();
    std::printf("[uav] magnetometer updates %s\n", ekf_.use_mag ? "ON" : "OFF (yaw free-runs)");
  }
}

void UavApp::update(float frame_dt, bool paused)
{
  if (paused) return;
  accumulator_ += frame_dt;
  int n = 0;
  while (accumulator_ >= kPhysicsDt && n++ < kMaxStepsPerFrame) {
    physics_step(kPhysicsDt);
    accumulator_ -= kPhysicsDt;
  }
  if (n >= kMaxStepsPerFrame) accumulator_ = 0.0f;

  // UPDATE THE ORIGINAL JET MODEL HERE
  if (plane_model_) {
    plane_model_->set_transform(fw_.position, fw_.attitude);
  }
}

void UavApp::physics_step(float dt)
{
  SharedBlock* blk = bridge_.data();
  const auto now = std::chrono::steady_clock::now();

  ControlCmd cmd{};
  if (blk) {
    cmd = blk->cmd;
    if (cmd.reset_request) {
      blk->cmd.reset_request = 0;
      reset();
      return;
    }
    if (cmd.seq != last_cmd_seq_) {
      last_cmd_seq_ = cmd.seq;
      last_cmd_time_ = now;
      cmds_received_++;
    }
  }
  const float since_cmd = std::chrono::duration<float>(now - last_cmd_time_).count();
  link_ok_ = blk && cmd.armed && last_cmd_seq_ != 0 && since_cmd < link_timeout_s;
  
  if (link_ok_ != link_was_ok_) {
    std::printf(link_ok_ ? "[uav] controller.py connected -- AUTO\n"
                         : "[uav] controller.py link lost -- FAILSAFE LEVEL FLIGHT\n");
    std::fflush(stdout);
    link_was_ok_ = link_ok_;
  }

  const float max_bank  = glm::radians(max_bank_deg);
  const float max_pitch = glm::radians(max_pitch_deg);
  float roll_cmd, pitch_cmd, target_speed;
  
  if (link_ok_) {
    roll_cmd     = std::clamp(cmd.roll_angle,  -max_bank,  max_bank);
    pitch_cmd    = std::clamp(cmd.pitch_angle, -max_pitch, max_pitch);
    target_speed = cmd.target_speed > 1.0f ? cmd.target_speed : FixedWing::cruise_speed();
  } else {
    roll_cmd     = 0.0f;
    pitch_cmd    = 0.0f;
    target_speed = FixedWing::cruise_speed();
  }

  const float speed_err = target_speed - fw_.airspeed();
  speed_integral_ += speed_err * dt;
  speed_integral_ = std::clamp(speed_integral_, -5.0f, 5.0f);
  float throttle = FixedWing::cruise_throttle() + 0.05f * speed_err + 0.01f * speed_integral_;
  throttle = std::clamp(throttle, 0.0f, 1.0f);
  last_throttle_ = throttle;

  // Inner loop: the stabilisation runs on the latest EKF output (computed from the IMU sample
  // of the previous step, as on the real autopilot), or on truth when E switched it off.
  FixedWing::InnerLoopFeedback fb;
  fb.roll = ekf_.estimate().roll;
  fb.pitch = ekf_.estimate().pitch;
  fb.rate_error = ekf_.rate_error_sim();
  fw_.step(roll_cmd, pitch_cmd, throttle, dt, inner_loop_on_ekf ? &fb : nullptr);
  sim_time_us_ += static_cast<uint64_t>(dt * 1e6f + 0.5f);
  steps_++;

  // Sample the IMU on the new state and run the STM32 firmware loop (imu_port_read + main.c)
  // (on a failed read imu keeps the previously published sample and est the last estimate)
  ImuSample imu = blk ? blk->imu : ImuSample{};
  EkfState est{};
  ekf_.step(fw_.attitude, fw_.ang_vel, fw_.accel_world, fw_.airspeed(), dt, sim_time_us_, imu, est);

  // Fly-through detection (hit margin is larger for the jet)
  const float dist = glm::length(fw_.position - target_->position()) - target_->radius();
  if (dist < hit_margin && blk) {
    blk->target.hits++;
    blk->target.last_hit_step = static_cast<uint32_t>(steps_);
    std::printf("[uav] TARGET HIT #%u at t=%.2f s  (EKF err now %.2f deg, mean %.2f, max %.2f)\n",
                blk->target.hits, sim_time_us_ * 1e-6, est.att_error_deg, ekf_.err_mean_deg(), ekf_.err_max_deg());
    std::fflush(stdout);
    float r, p, heading;
    frames::euler_aircraft(frames::sim_to_flu(fw_.attitude), r, p, heading);
    target_->respawn(fw_.position, heading, 250.0f, glm::radians(45.0f), 25.0f);
  }

  if (blk) {
    blk->imu = imu;
    blk->ekf = est;
    publish_truth();
    blk->header.sim_time_us = sim_time_us_;
    blk->header.physics_steps = steps_;
  }
}

void UavApp::publish_truth()
{
  SharedBlock* blk = bridge_.data();
  if (!blk) return;

  const glm::quat q = frames::sim_to_flu(fw_.attitude);
  SimState& s = blk->truth;
  s.attitude_q[0] = q.w;
  s.attitude_q[1] = q.x;
  s.attitude_q[2] = q.y;
  s.attitude_q[3] = q.z;
  frames::euler_aircraft(q, s.roll, s.pitch, s.yaw);
  const glm::vec3 w = frames::sim_to_flu(fw_.ang_vel);
  for (int i = 0; i < 3; i++) {
    s.ang_vel[i] = w[i];
    s.position[i] = fw_.position[i];
    s.velocity[i] = fw_.velocity[i];
    blk->target.position[i] = target_->position()[i];
  }
  s.airspeed = fw_.airspeed();
  s.altitude = fw_.position.y;
  blk->target.radius = target_->radius();
  blk->target.distance = glm::length(fw_.position - target_->position()) - target_->radius();
}

glm::quat UavApp::camera_rotation() const
{
  return fw_.attitude * glm::angleAxis(glm::radians(-90.0f), glm::vec3(0, 1, 0)) *
         glm::angleAxis(glm::radians(camera_tilt_deg), glm::vec3(1, 0, 0));
}

void UavApp::render(gfx::Renderer& renderer, gfx::Object3D& scene, gfx::Camera& window_camera, bool orbit)
{
  onboard_cam_->set_position(fw_.position);
  onboard_cam_->set_rotation_quat(camera_rotation());

  // Hide the jet from the onboard camera
  if (plane_model_) plane_model_->visible = false;
  
  renderer.render(*onboard_cam_, scene);
  
  if (bridge_.ok()) {
    SharedBlock* blk = bridge_.data();
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, std::min(win_w_, CAM_WIDTH), std::min(win_h_, CAM_HEIGHT), GL_RGB, GL_UNSIGNED_BYTE,
                 blk->pixels);
    blk->camera.frame_id++;
  }

  // Bring the jet back for the window view
  if (plane_model_) plane_model_->visible = true;

  if (!orbit && view_ == View::Onboard) {
    if (plane_model_) plane_model_->visible = false;
    return; 
  }
  
  if (!orbit) {
    glm::vec3 fwd = fw_.attitude * glm::vec3(1, 0, 0);
    fwd.y = 0.0f;
    fwd = glm::length(fwd) > 1e-3f ? glm::normalize(fwd) : glm::vec3(1, 0, 0);
    
    // Pull the chase camera back a bit so the large jet fits on screen
    window_camera.set_position(fw_.position - fwd * 20.0f + glm::vec3(0, 6.0f, 0));
    window_camera.look_at(fw_.position + fwd * 15.0f);
  }
  
  renderer.render(window_camera, scene);
}

void UavApp::draw_hud(float fps)
{
  SharedBlock* blk = bridge_.data();
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
  ImGui::SetNextWindowPos(ImVec2(10, 10));
  ImGui::SetNextWindowSize(ImVec2(260, 340));
  ImGui::SetNextWindowBgAlpha(0.45f);
  ImGui::Begin("UAV", nullptr, flags);
  ImGui::Text("MODE:  %s", link_ok_ ? "AUTO (controller.py)" : "FAILSAFE LEVEL FLIGHT");
  ImGui::Text("ALT:   %.1f m   SPD %.1f m/s", fw_.position.y, fw_.airspeed());
  ImGui::Text("THR:   %.0f %%", last_throttle_ * 100.0f);
  
  if (!link_ok_) {
    if (!blk)
      ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "shared memory FAILED");
    else if (cmds_received_ == 0)
      ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "waiting for controller.py");
    else
      ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "controller.py stopped");
  }
  
  if (blk) {
    const SimState& t = blk->truth;
    const EkfState& e = blk->ekf;
    ImGui::Separator();
    ImGui::Text("        truth    EKF");
    ImGui::Text("ROLL  %6.1f  %6.1f", glm::degrees(t.roll), glm::degrees(e.roll));
    ImGui::Text("PITCH %6.1f  %6.1f", glm::degrees(t.pitch), glm::degrees(e.pitch));
    ImGui::Text("YAW   %6.1f  %6.1f", glm::degrees(t.yaw), glm::degrees(e.yaw));
    ImGui::Text("EKF err %.2f deg  acc %s", e.att_error_deg, e.accel_used ? "used" : "gated");
    ImGui::Text("err mean %.2f  max %.2f deg", ekf_.err_mean_deg(), ekf_.err_max_deg());
    ImGui::Text("accel comp [C]: %s  used %.0f%%", ekf_.accel_compensation ? "ON" : "OFF", ekf_.accel_used_pct());
    ImGui::Text("bias  %.2f %.2f %.2f dps", glm::degrees(e.bias[0]), glm::degrees(e.bias[1]),
                glm::degrees(e.bias[2]));
    ImGui::Text("inner loop [E]: %s", inner_loop_on_ekf ? "EKF" : "truth");
    ImGui::Text("mag [M]: %s  cal %s", ekf_.use_mag ? "ON" : "OFF", ekf_.mag_cal_valid() ? "OK" : "none");
    ImGui::Separator();
    ImGui::Text("TARGET %.1f m   HITS %u", blk->target.distance, blk->target.hits);
  }
  ImGui::Text("FPS:   %.1f   view: %s", fps, view_ == View::Chase ? "chase" : "onboard");
  ImGui::End();
}
