# Hackathon UAV mode (fixed-wing)

The flight sim flies a small fixed-wing UAV on the complete STM32 attitude stack from
`stm32_utilities/43-STM32Fxxx_MPU9250 _EKF/User`: the MEKF (`ekf.c`), the hard/soft-iron
calibration (`magcal.c`) and its Gauss-Jordan solver (`Gauss_elimination.c`), all compiled
**unmodified**. The EKF closes **both** loops: the sim's stabilisation loop tracks bank/pitch
against the EKF estimate and regulates the bias-corrected gyro, and `controller.py` flies into a
magenta target sphere using only the onboard camera and the EKF attitude. After every hit the sim respawns the target 250 m ahead (±45°, ±25 m), and
the controller keeps going after the next one, so the flight is **continuous**.
Set `#define UAV_MODE 0` in `src/main.cpp` to get the original jet back.

```
 main.cpp ──> UavApp (src/hackathon/uav_app.cpp)
               ├─ FixedWing       100 Hz rigid body: angle commands + speed setpoint
               ├─ EkfAdapter      imu_port.c + main.c: simulated MPU9250/AK8963 -> magcal -> ekf.c
               │                  (inner loop of FixedWing flies on its output)
               ├─ Target          magenta sphere, respawns after each hit
               ├─ onboard camera  rendered every frame, glReadPixels -> shared memory
               └─ SimBridge       named shared memory "flightsim_bridge" (Windows + Linux)
                                        ▲ commands      │ image + EKF attitude
                                        │               ▼
                                     controller.py (OpenCV + EKF-based guidance)
```

## Build

**Visual Studio:** open `OpenGL_Flightsim.sln`, build Release x64.
**Linux:** `mkdir build && cd build && cmake .. && cmake --build .`
This builds `flightsim` (the 3D sim) and `flightsim_headless` (see *Testing the EKF*).

## Run

1. Start the simulator. Until a controller connects it flies wings-level (`FAILSAFE LEVEL FLIGHT`).
2. In a second terminal:
   ```
   pip install opencv-python numpy
   python controller.py --show
   ```
   The HUD switches to `AUTO`. Every hit is printed in both consoles with the EKF error statistics.

Keys in the sim: **R** reset (reruns the pre-flight), **V** chase/onboard view, **C** EKF accel
compensation on/off, **E** inner loop on EKF / truth, **M** magnetometer updates on/off
(`EKF_USE_MAG`), **O** orbit camera, **P** pause.

Controller options:

| option | meaning |
|---|---|
| `--show` | camera window with detection overlay |
| `--hits N` | stop after N hits (default: run forever) |
| `--max-time S` | stop after S seconds of sim time |
| `--reset` | reset UAV + target at start |
| `--attitude ekf\|truth` | attitude used for guidance (truth = baseline to compare the EKF against) |
| `--log run.csv` | per-step CSV: truth vs EKF roll/pitch/yaw, error, accel gate, guidance |

## The STM32 pipeline in the sim

`ekf_adapter.cpp` replaces only the two hardware files of the firmware, `imu_port.c` and `main.c`:

* **Sensors (`imu_port_read`).** Gyro and accel are quantised to the MPU9250 LSB (±500 dps, ±4 g)
  with white noise and a ~1 °/s constant gyro bias. The magnetometer is produced in the AK8963's
  own axes, in counts, with a hard-iron offset (12, −8, 20 µT) and soft-iron gains
  (1.10, 0.92, 1.04), and converts at **8 Hz**. It is then remapped exactly like `read_mag_raw()`
  (body X = mag Y, body Y = mag X, body Z = −mag Z), scaled by `MMult` to 0.1 µT and passed
  through `magcal_apply()` once a calibration exists. `mag_new` is set only on a fresh
  conversion, and `dt` is gated to (0, 0.5) s. With `--i2c-fail P` a read can fail: the sample
  is dropped and the next `dt` spans the gap, as on the board.
* **Filter loop (`main.c`).** `ekf_predict` runs on every sample and `ekf_update_accel` on every
  sample (the 1 g gate is inside it). `ekf_update_mag` runs only when `mag_new`, only if
  `EKF_USE_MAG`, and only with a valid calibration. The tuning is the same as `main.c`.
* **Pre-flight, on every reset:**
  1. *Magnetometer calibration.* `imu_port_cal_begin` / `feed` (400 fresh samples while the
     airframe is turned through all orientations) / `finish` → `magcal_solve`. The recovered
     offset and scale are checked against the injected distortion and printed.
  2. *Stationary alignment.* The aircraft sits still on the launch rail for `--align` seconds
     (default 10). Roll and pitch lock to gravity, heading locks to the field, and the gyro bias
     converges before launch.
* **Inner loop on the EKF.** `FixedWing::step` takes an `InnerLoopFeedback`. The angle loop uses
  the EKF roll/pitch, and the rate loop regulates the bias-corrected gyro, so the true rate is
  command − (gyro error). An attitude error becomes a real bank error, and a residual bias becomes
  a real rotation. **E** / `--inner truth` switches back to ideal sensors.

## Testing the EKF

The guidance converts the target pixel into a line-of-sight ray and rotates it into the level
frame **with the EKF quaternion**. A wrong attitude estimate therefore produces a wrong bearing and
elevation, and the aircraft misses. That makes the whole flight an end-to-end test of the filter.

Headless (no window, fast, works in CI):
```
./flightsim_headless --speedup 4
python controller.py --log run.csv
```
Headless options: `--inner ekf|truth`, `--align S`, `--no-magcal`, `--no-mag`, `--no-comp`,
`--mag-hz F`, `--i2c-fail P`, `--time S`, `--speedup X`.

Reference results (180 s of sim time, headless, EKF guidance in `controller.py`):

| configuration | hits | EKF error mean / max |
|---|---|---|
| **full stack: inner loop on EKF, magcal, 10 s alignment** | 10 | **0.30° / 1.35°** |
| same, inner loop on truth (`--inner truth`) | 11 | 0.33° / 1.10° |
| no alignment (`--align 0`), starts with 1.5 °/s bias error | 7 | 0.47° / 2.9° |
| 5 % I2C read failures (`--i2c-fail 0.05`) | 10 | 0.33° / 1.11° |
| no magnetometer (`--no-mag`): yaw and z-bias unobservable | 8 | 5.7° / 19.6° (mostly yaw) |
| no calibration (`--no-magcal`): mag gated off like `EKF_USE_MAG` | 9 | 7.2° / 28° |
| **accel compensation OFF (`--no-comp`)** | **0** | 77° / 180° (control lost) |
| accel compensation OFF, inner loop on truth | 0 | 32° / 64° |
| accel compensation OFF, mag at 50 Hz (`--mag-hz 50`) | 7 | 12.8° / 56° |

Pre-flight: magcal recovers the hard-iron offset to 0.06 µT and the soft-iron scale to 0.7 %, the
calibrated |m| is 1 ± 0.02, and after alignment the attitude error is 0.3° and the bias error 0.05 °/s.

What this shows:

* With the full stack, the EKF can fly the aircraft. It hits as often as the truth-fed inner loop,
  and the error stays under 1.4° all flight.
* Without fixed-wing accel compensation, the uncompensated filter pulls the estimate towards
  wings-level in every turn. Once the inner loop runs on that estimate, the autopilot banks further
  to "correct" it and control is lost. With the old truth-fed inner loop this could not happen.
* The magnetometer rate matters. At the AK8963's real 8 Hz, heading aiding is too weak to hold an
  uncompensated filter together. At 50 Hz (the old setting) it limps along.

## What was fixed

* **Shared-memory offsets (controller.py).** `CameraInfo` starts with a `uint64_t`, so the
  compiler puts 4 bytes of padding after `TargetInfo`. The old controller read the camera info and
  every pixel 4 bytes off, and its size check failed. The offsets are now asserted in `sim_types.h`.
* **Windows support (controller.py).** The old controller only opened `/dev/shm`. It now opens the
  named file mapping on Windows and waits for the sim to start.
* **Continuous flight (controller.py).** The old controller `return`ed after the first "hit" (judged
  by blob area). It now follows the sim's hit counter and keeps flying. If it overshoots, it loops
  back towards the target.
* **Target too small to detect.** At 250 m the sphere is only about 8 px wide, and two erode passes
  removed it. Detection now uses a light opening.
* **Guidance uses the EKF.** Bank and pitch commands now come from the line of sight in the level
  frame, computed from the EKF attitude, instead of raw image x/y. Raw image x/y is wrong whenever
  the aircraft is banked.
* **Flight model (fixed_wing.h).** The nose never yawed in a bank, so the velocity turned while
  the nose and camera kept pointing the old way. Lift also ignored angle of attack. The model now has
  coordinated turns (ψ̇ = g·tanφ / V), CL(α) lift with induced drag and stall, and sideslip damping.
* **EKF on a fixed-wing (ekf_adapter.cpp).** In a turn, the accelerometer reads gravity plus the
  centripetal acceleration. Below the 1.15 g gate, the filter is pulled towards wings-level. The
  adapter subtracts ω×V and dV/dt (airspeed-aided, as an autopilot does) before calling
  `ekf_update_accel`. `ekf.c` itself is unchanged. Press **C** to compare.
* The jet model is now attached to the UAV in the chase view, and the VS project references
  `fixed_wing.h` instead of the deleted `quadcopter.h`.

## Files

| File | What it does |
|---|---|
| `src/hackathon/sim_types.h` | Shared-memory layout + offset asserts. If you change it, also change `controller.py` and bump `BRIDGE_VERSION`. |
| `src/hackathon/fixed_wing.h` | Fixed-wing physics in the sim's y-up frame; inner loop on `InnerLoopFeedback` (EKF) or truth |
| `src/hackathon/sim_bridge.*` | `CreateFileMapping` (Windows) / `shm_open` (Linux) |
| `src/hackathon/ekf.c`, `ekf.h` | **Unmodified** copy from `stm32_utilities/43-STM32Fxxx_MPU9250 _EKF/User` |
| `src/hackathon/magcal.c`, `magcal.h` | **Unmodified** ellipsoid hard/soft-iron calibration |
| `src/hackathon/Gauss_elimination.c`, `.h` | **Unmodified** Gauss-Jordan solver used by `magcal_solve` |
| `src/hackathon/ekf_adapter.*` | `imu_port.c` + `main.c` for the sim: MPU9250/AK8963 model, magcal lifecycle, alignment, accel compensation, frame conversion |
| `src/hackathon/target.*` | Target sphere (gfx `Basic` material, pure magenta) |
| `src/hackathon/uav_app.*` | Main-loop integration and HUD |
| `src/hackathon/headless_sim.cpp` | Window-less test harness with a synthetic camera |
| `controller.py` | Vision, EKF-based line-of-sight guidance, logging |
