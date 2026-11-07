#pragma once
// Flat-shaded magenta sphere the UAV has to hit. Drawn through the simulator's own
// gfx::Mesh + gfx::Basic material, so it is part of the normal scene graph.

#include <memory>
#include <random>

#include "../gfx.h"

class Target
{
 public:
  explicit Target(float radius = 1.5f);

  gfx::Mesh& mesh() { return *mesh_; }

  void set_position(const glm::vec3& p);
  const glm::vec3& position() const { return pos_; }
  float radius() const { return radius_; }

  // Places the target `distance` m from `from` within +-`bearing_spread` rad of `heading`
  // (heading 0 = sim +x, positive = toward +z / right) and +-`dh` m in height.
  void respawn(const glm::vec3& from, float heading, float distance, float bearing_spread, float dh);

 private:
  std::unique_ptr<gfx::Mesh> mesh_;
  glm::vec3 pos_{0.0f};
  float radius_;
  std::mt19937 rng_{42};
};
