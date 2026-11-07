# EKF Flight Simulator: Autonomous UAV Hackathon Project

This repository is an extension of the excellent [gue-ni/OpenGL_Flightsim](https://github.com/gue-ni/OpenGL_Flightsim) project. 

### About the Base Project
The original **OpenGL_Flightsim** is a simple but realistic-looking flight simulator built from scratch in C++ using OpenGL, SDL2, and GLM. It features a full fixed-wing aerodynamics model, realistic terrain rendering, and joystick/keyboard controls. It serves as the perfect sandbox for testing flight control algorithms.

### The Hackathon Integration
For this project, the simulator was modified to create an autonomous UAV mission:
*   **Fixed-Wing UAV Physics:** The aircraft is controlled via high-level Roll/Pitch commands.
*   **STM32 EKF Implementation:** An unmodified Multiplicative Extended Kalman Filter (MEKF) in C (`ekf.c`) estimates the aircraft's attitude using synthetic noisy IMU data.
*   **Computer Vision Guidance:** A Python script (`controller.py`) uses OpenCV to detect a magenta target from the onboard camera and calculates the 3D Line-of-Sight using the EKF's attitude.
*   **Shared Memory Bridge:** A C++ struct (`sim_types.h`) enables blazing-fast, zero-latency communication between the C++ simulator, the C EKF, and the Python controller.

---

## 🏗️ Software Architecture

The system is highly modular, with three main processes communicating via shared memory. 

*(Insert the architecture image here: `![Software Architecture](Software_architecture.png)`)*

### Interaction Flow (as depicted in the architecture):
1. **True Position:** The C++ Physics engine (`fixed_wing.h`) moves the airplane.
2. **Noisy IMU Data:** The IMU Synthesizer generates noisy gyro/accel data and writes it to **Shared Memory**.
3. **C EKF (The Senses):** Reads the noisy IMU data, runs the Kalman Filter, and writes the **EKF Attitude** back to Shared Memory.
4. **Camera Renderer:** Captures the raw image from the onboard camera and writes it to Shared Memory.
5. **Python Controller (The Brain):** Reads the image and the EKF Attitude. It detects the target using OpenCV, calculates the Line-of-Sight math, and writes **Control Commands** to Shared Memory.
6. **C++ Physics (The Airplane):** Reads the Roll/Pitch commands and applies aerodynamic forces, relying on the EKF Attitude to execute the maneuver.

---

## 📁 Repository Structure

```text
OpenGL_Flightsim/
├── CMakeLists.txt                  # Build script: Compiles the GUI sim & headless test runner
├── HACKATHON_README.md             # Documentation: Explains the EKF integration and how to run it
├── controller.py                   # THE BRAIN: Python vision, target tracking, and LOS guidance
├── build/                          # [GENERATED] CMake cache, .o files, and compiled binaries
│
├── assets/                         # SIMULATOR RESOURCES
│   ├── data/                       # Aerodynamic tables (Lift/Drag data)
│   ├── models/                     # 3D Meshes (Falcon jet, Cessna)
│   └── textures/                   # Visuals (skybox, terrain, HUD sprites, jet skin)
│
├── lib/                            # THIRD-PARTY LIBRARIES (Single-header)
│   ├── imgui/                      # Dear ImGui: Draws the HUD overlay
│   ├── stb_image.h                 # Loads .jpg/.png textures
│   └── tiny_obj_loader.h           # Loads 3D models
│
├── shaders/                        # OPENGL RENDERING (Runs on the GPU)
│   ├── *.vert / *.frag             # Shaders for lighting, terrain, skybox, HUD
│
└── src/                            # SOURCE CODE
    ├── main.cpp                    # App entry point. Sets up window, loads assets, calls UavApp
    ├── gfx.cpp / gfx.h             # Graphics engine: Handles cameras, meshes, rendering
    ├── flightmodel.h               # The ORIGINAL complex fixed-wing aerodynamics
    ├── phi.h / pid.h / terrain.h   # Original math, PID controllers, and terrain logic
    │
    └── hackathon/                  # THE CORE HACKATHON INTEGRATION
        ├── uav_app.cpp / uav_app.h # ORCHESTRATOR: Runs the 100Hz loop, camera capture, shared memory
        ├── fixed_wing.h            # CUSTOM PHYSICS: Aerodynamics that rely on the EKF to fly
        ├── ekf_adapter.cpp / .h    # BRIDGE: Generates synthetic noisy IMU data for the EKF
        ├── ekf.c / ekf.h           # THE FILTER: Unmodified C Multiplicative Extended Kalman Filter
        ├── magcal.c / magcal.h     # Unmodified C Magnetometer Calibration
        ├── Gauss_elimination.c/.h  # Unmodified C Matrix Math solver
        ├── sim_bridge.cpp / .h     # SHARED MEMORY: Allows C++ and Python to talk instantly
        ├── sim_types.h             # LAYOUT: The exact byte structure of the shared memory
        ├── target.cpp / target.h   # TARGET: Renders and respawns the magenta sphere
        └── headless_sim.cpp        # TEST RUNNER: Runs physics + EKF without the GUI (for CI/testing)
