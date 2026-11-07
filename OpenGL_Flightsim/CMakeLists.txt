cmake_minimum_required(VERSION 3.10)

project(OpenGL_Flightsim C CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(GLEW REQUIRED)
find_package(SDL2 REQUIRED)

add_executable(flightsim
    # Source files
    src/ai.h
    src/terrain.h
    src/collider.h
    src/data.h
    src/flightmodel.h
    src/gfx.cpp
    src/gfx.h
    src/main.cpp
    src/phi.h
    src/pid.h
    # hackathon: fixed_wing + STM32 EKF + shared-memory bridge
    src/hackathon/sim_types.h
    src/hackathon/fixed_wing.h
    src/hackathon/sim_bridge.h
    src/hackathon/sim_bridge.cpp
    src/hackathon/ekf.h
    src/hackathon/ekf.c
    src/hackathon/magcal.h
    src/hackathon/magcal.c
    src/hackathon/Gauss_elimination.h
    src/hackathon/Gauss_elimination.c
    src/hackathon/ekf_adapter.h
    src/hackathon/ekf_adapter.cpp
    src/hackathon/target.h
    src/hackathon/target.cpp
    src/hackathon/uav_app.h
    src/hackathon/uav_app.cpp
    lib/stb_image.h
    lib/tiny_obj_loader.h
    lib/imgui/imconfig.h
    lib/imgui/imgui_demo.cpp
    lib/imgui/imgui.cpp
    lib/imgui/imgui_draw.cpp
    lib/imgui/imgui.h
    lib/imgui/imgui_impl_opengl3.cpp
    lib/imgui/imgui_impl_opengl3.h
    lib/imgui/imgui_impl_opengl3_loader.h
    lib/imgui/imgui_impl_sdl2.cpp
    lib/imgui/imgui_impl_sdl2.h
    lib/imgui/imgui_internal.h
    lib/imgui/imgui_tables.cpp
    lib/imgui/imgui_widgets.cpp
    lib/imgui/imstb_rectpack.h
    lib/imgui/imstb_textedit.h
    lib/imgui/imstb_truetype.h
)

target_include_directories(flightsim PUBLIC
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(flightsim
    SDL2
    GLEW
    GL
    GLU
)

if(UNIX AND NOT APPLE)
    target_link_libraries(flightsim rt m)  # shm_open
endif()

# Headless test harness: same physics + STM32 EKF + shared memory, synthetic camera, no window.
#   ./flightsim_headless [--time S] [--speedup X] [--no-comp] [--inner ekf|truth] [--align S] [--no-magcal]   then   python controller.py
add_executable(flightsim_headless
    src/hackathon/headless_sim.cpp
    src/hackathon/ekf.c
    src/hackathon/magcal.c
    src/hackathon/Gauss_elimination.c
    src/hackathon/ekf_adapter.cpp
    src/hackathon/sim_bridge.cpp
)
if(UNIX AND NOT APPLE)
    target_link_libraries(flightsim_headless rt m)
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
