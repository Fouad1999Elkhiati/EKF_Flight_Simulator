#include "target.h"

#include <cmath>
#include <vector>

static std::shared_ptr<gfx::Geometry> make_sphere(int rings, int sectors)
{
  const float PI = 3.14159265358979f;
  auto vertex = [&](int r, int s, std::vector<float>& out) {
    const float phi = PI * static_cast<float>(r) / rings;
    const float theta = 2.0f * PI * static_cast<float>(s) / sectors;
    const glm::vec3 p(std::sin(phi) * std::cos(theta), std::cos(phi), std::sin(phi) * std::sin(theta));
    out.insert(out.end(), {p.x, p.y, p.z, p.x, p.y, p.z, float(s) / sectors, float(r) / rings});
  };

  std::vector<float> v;
  v.reserve(static_cast<size_t>(rings) * sectors * 6 * 8);
  for (int r = 0; r < rings; r++) {
    for (int s = 0; s < sectors; s++) {
      vertex(r, s, v);
      vertex(r + 1, s, v);
      vertex(r + 1, s + 1, v);
      vertex(r, s, v);
      vertex(r + 1, s + 1, v);
      vertex(r, s + 1, v);
    }
  }
  return std::make_shared<gfx::Geometry>(v, gfx::Geometry::POS_NORM_UV);
}

Target::Target(float radius) : radius_(radius)
{
  auto material = std::make_shared<gfx::Basic>(glm::vec3(1.0f, 0.0f, 1.0f));  // pure magenta
  mesh_ = std::make_unique<gfx::Mesh>(make_sphere(16, 32), material);
  mesh_->set_scale(glm::vec3(radius_));
  mesh_->receive_shadow = false;
}

void Target::set_position(const glm::vec3& p)
{
  pos_ = p;
  mesh_->set_position(p);
}

void Target::respawn(const glm::vec3& from, float heading, float distance, float bearing_spread, float dh)
{
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  const float b = heading + u(rng_) * bearing_spread;
  set_position(from + glm::vec3(std::cos(b) * distance, u(rng_) * dh, std::sin(b) * distance));
}
