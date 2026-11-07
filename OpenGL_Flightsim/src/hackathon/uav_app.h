#pragma once
// FIXED-WING UAV mode for OpenGL_Flightsim
#include <chrono>
#include <memory>

#include "../gfx.h"
#include "ekf_adapter.h"
#include "fixed_wing.h"
#include "sim_bridge.h"
#include "target.h"

class UavApp
{
 public:
  enum class View { Chase, Onboard };

  UavApp();
  bool init(gfx::Object3D& scene, int window_w, int window_h);
  
  // NEW: Pass the original loaded airplane model here so we can move it
  void attach_model(gfx::Object3D* model) { plane_model_ = model; }
  
  void handle_key(int sdl_keycode);
  void update(float frame_dt, bool paused);
  void render(gfx::Renderer& renderer, gfx::Object3D& scene, gfx::Camera& window_camera, bool orbit);
  void draw_hud(float fps);

  const glm::vec3& position() const { return fw_.position; }
  const glm::quat& attitude() const { return fw_.attitude; } // NEW: getter for main.cpp

  // Tunables
  glm::vec3 start_position{0.0f, 3000.0f, 0.0f};
  float camera_vfov_deg = 60.0f;
  float camera_tilt_deg = 15.0f; 
  float hit_margin = 2.0f;         // Larger margin for the jet's wingspan
  float link_timeout_s = 0.5f;     
  float max_bank_deg = 45.0f;      
  float max_pitch_deg = 25.0f;
  // Inner (stabilisation) loop feedback: true = fly on the STM32 EKF estimate and the
  // bias-corrected gyro, false = ideal sensors (truth). Toggle with E.
  bool inner_loop_on_ekf = true;

 private:
  void physics_step(float dt);
  void reset();
  void publish_truth();
  glm::quat camera_rotation() const;

  SimBridge bridge_;
  FixedWing fw_;
  EkfAdapter ekf_;
  std::unique_ptr<Target> target_;
  std::unique_ptr<gfx::Camera> onboard_cam_;
  
  gfx::Object3D* plane_model_ = nullptr; // Pointer to the original jet model

  View view_ = View::Chase;
  int win_w_ = 0, win_h_ = 0;
  float accumulator_ = 0.0f;
  uint64_t sim_time_us_ = 0;
  uint64_t steps_ = 0;
  uint64_t last_cmd_seq_ = 0;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  bool link_ok_ = false;
  bool link_was_ok_ = false;
  uint64_t cmds_received_ = 0;
  float last_throttle_ = 0.0f;
  float speed_integral_ = 0.0f;
};
