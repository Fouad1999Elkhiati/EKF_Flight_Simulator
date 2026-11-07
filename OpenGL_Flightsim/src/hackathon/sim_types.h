#pragma once
// Shared-memory layout between the simulator (C++) and controller.py.
//
// RULES
//  * Plain C types only (float / uint32_t / uint64_t) so the layout is identical
//    under MSVC, GCC and Python ctypes. glm types are NOT used here.
//  * If you change anything in this file, bump BRIDGE_VERSION and mirror the
//    change in controller.py (class SharedBlock). The controller refuses to run
//    when version or total size disagree.
//
// FRAME CONVENTIONS (used everywhere in this file)
//  * Attitude angles (truth and EKF) are aircraft-style:
//      roll  > 0 : right wing down
//      pitch > 0 : nose up
//      yaw   > 0 : nose right  (heading, clockwise seen from above)
//  * Quaternions are w,x,y,z, body -> nav, in the EKF's FLU/ENU-like frame
//      (x forward, y left, z up) -- the same frame your STM32 ekf.c uses.
//  * IMU vectors are in that same FLU body frame:
//      gyro  [rad/s], accel [g] (reads +1 g on z when level), mag [unit vector]
//  * ControlCmd uses ANGLE commands + speed setpoint (fixed-wing):
//      roll_angle  > 0 : right wing down (bank right)
//      pitch_angle > 0 : nose up
//      target_speed    : desired airspeed in m/s (PI controller runs in the sim)

#include <cstddef>
#include <cstdint>

#define BRIDGE_MAGIC   0x55415653u  // 'UAVS'
#define BRIDGE_VERSION 3u           // bumped: ControlCmd semantics changed for fixed-wing
#define BRIDGE_NAME    "flightsim_bridge"

#define CAM_WIDTH    640
#define CAM_HEIGHT   480
#define CAM_CHANNELS 3

struct BridgeHeader {
  uint32_t magic;           // BRIDGE_MAGIC once the simulator has initialised the block
  uint32_t version;         // BRIDGE_VERSION
  uint32_t total_size;      // sizeof(SharedBlock)
  uint32_t sim_running;     // 1 while the simulator is alive, 0 after it exits
  uint64_t sim_time_us;     // simulated time
  uint64_t physics_steps;   // incremented every 100 Hz physics step
};

// Ground truth. For logging/scoring only -- a camera-only controller must not use it.
struct SimState {
  float attitude_q[4];      // w,x,y,z (FLU, body -> nav)
  float roll, pitch, yaw;   // rad
  float ang_vel[3];         // FLU body rates, rad/s
  float position[3];        // simulator world frame (x fwd/north, y up, z right/east), m
  float velocity[3];        // simulator world frame, m/s
  float airspeed;           // m/s
  float altitude;           // m (== position[1])
};

struct ImuSample {
  float gyro[3];            // rad/s, includes bias + noise
  float accel[3];           // g, specific force
  float mag[3];             // unit-ish vector
  uint32_t mag_new;         // 1 when mag holds a fresh sample this step
  uint64_t timestamp_us;
};

// Output of your STM32 ekf.c, running inside the simulator at 100 Hz.
struct EkfState {
  float q[4];               // w,x,y,z (FLU, body -> nav)
  float roll, pitch, yaw;   // rad, aircraft convention (see top)
  float bias[3];            // estimated gyro bias, rad/s (FLU)
  float att_error_deg;      // angle between EKF and true attitude (diagnostic)
  uint32_t accel_used;      // 1 if the last accel update was accepted (|a| ~ 1 g)
};

// Written by controller.py. FIXED-WING: angle commands + speed setpoint.
// The simulator runs the throttle PI loop internally, so the Python side only
// needs to say "how fast should I be going".
//
//   seq           increment on every write; the sim ignores commands whose seq
//                 has not changed for link_timeout_s
//   roll_angle    rad, + = right wing down (bank right)
//   pitch_angle   rad, + = nose up
//   target_speed  m/s, PI-controlled throttle
//   thrust_override 0..1, used only if target_speed <= 0 (manual override)
//   armed         0 = ignore command (failsafe level flight), 1 = fly it
//   reset_request set to 1 to ask the sim to reset UAV + target; sim clears it
struct ControlCmd {
  uint64_t seq;
  float roll_angle;
  float pitch_angle;
  float target_speed;
  float thrust_override;
  uint32_t armed;
  uint32_t reset_request;
};

struct TargetInfo {
  float position[3];        // world frame (truth, scoring only)
  float radius;             // m
  float distance;           // truth distance UAV -> target surface, m (scoring only)
  uint32_t hits;            // number of hits so far
  uint32_t last_hit_step;   // header.physics_steps at the last hit
};

struct CameraInfo {
  uint64_t frame_id;        // incremented AFTER pixels are fully written
  uint32_t width, height, channels;
  float vfov_deg;           // vertical field of view
  float tilt_deg;           // camera tilted up from the body x axis by this much
  uint32_t bottom_up;       // 1: row 0 is the bottom of the image (OpenGL), flip it
  uint32_t rgb;             // 1: pixel order R,G,B
  uint32_t _pad;
};

struct SharedBlock {
  BridgeHeader header;
  SimState     truth;
  ImuSample    imu;
  EkfState     ekf;
  ControlCmd   cmd;
  TargetInfo   target;
  CameraInfo   camera;
  uint8_t      pixels[CAM_WIDTH * CAM_HEIGHT * CAM_CHANNELS];
};

// Layout guards -- the Python side checks the same numbers.
static_assert(sizeof(BridgeHeader) == 32, "BridgeHeader layout");
static_assert(sizeof(SimState) == 72, "SimState layout");
static_assert(sizeof(ImuSample) == 48, "ImuSample layout");
static_assert(sizeof(EkfState) == 48, "EkfState layout");
static_assert(sizeof(ControlCmd) == 32, "ControlCmd layout");
static_assert(sizeof(TargetInfo) == 28, "TargetInfo layout");
static_assert(sizeof(CameraInfo) == 40, "CameraInfo layout");
// Byte offsets inside SharedBlock (controller.py hard-codes these). Note the 4 bytes of
// padding between TargetInfo (ends at 260) and CameraInfo (8-byte aligned, starts at 264).
static_assert(offsetof(SharedBlock, truth) == 32, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, imu) == 104, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, ekf) == 152, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, cmd) == 200, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, target) == 232, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, camera) == 264, "SharedBlock offsets");
static_assert(offsetof(SharedBlock, pixels) == 304, "SharedBlock offsets");
static_assert(sizeof(SharedBlock) == 304 + CAM_WIDTH * CAM_HEIGHT * CAM_CHANNELS, "SharedBlock size");
